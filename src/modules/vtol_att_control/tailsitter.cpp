/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "vtol_att_control_main.h"
#include <systemlib/mavlink_log.h>

#ifndef M_PI
#define M_PI (3.14159265f)
#endif

#define RAD_TO_DEG(x) ((x) / 3.1416f * 180.0f)
#define DEG_TO_RAD(x) ((x) / 180.0f * 3.1416f)

#define ARSP_YAW_CTRL_DISABLE     (4.0f)	// airspeed at which we stop controlling yaw during a front transition
#define THROTTLE_TRANSITION_MAX   (0.25f)	// maximum added thrust above last value in transition
#define PITCH_TRANSITION_FRONT_P1 (-_params->front_trans_pitch_sp_p1)	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_BACK     (-0.25f)	// pitch angle to switch to MC

static  orb_advert_t mavlink_log_pub = nullptr;

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.f_trans_start_t = 0.0f;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tailsitter.front_trans_dur_p2 = param_find("F_TRANS_DUR_P2");
	_params_handles_tailsitter.sys_ident_input    = param_find("SYS_IDENT_INPUT");
	_params_handles_tailsitter.sys_ident_num      = param_find("SYS_IDENT_NUM");
}

void
Tailsitter::parameters_update()
{
	float v;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tailsitter.front_trans_dur_p2, &v);
	_params_tailsitter.front_trans_dur_p2 = v;

	param_get(_params_handles_tailsitter.sys_ident_input, &v);
	_params_tailsitter.sys_ident_input = v;

	param_get(_params_handles_tailsitter.sys_ident_num, &v);
	_params_tailsitter.sys_ident_num = v;

	/* update the CL points */
	int iden_num = 0;
	if ((_params_tailsitter.sys_ident_num >= 9) && (_params_tailsitter.sys_ident_num >= 0)) {
		iden_num = _params_tailsitter.sys_ident_num;
	}

	memcpy(_CL_Degree, CL_SYS_ID[iden_num], sizeof(_CL_Degree));
}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/
	_euler = matrix::Quatf(_v_att->q);//Eulerf _euler = Quatf(_v_att->q);
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.f_trans_start_t) * 1e-6f;
	//static bool _lock_to_mc = 0;
	//float pitch = _euler.theta();

	if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
			//_lock_to_mc = 0;
			_vtol_schedule.b_trans_start_t = hrt_absolute_time();
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= MC_MODE;
			_vtol_schedule.b_trans_start_t = hrt_absolute_time();
			break;

		case TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			_vtol_schedule.b_trans_start_t = hrt_absolute_time();
			break;

		case TRANSITION_BACK:
			time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.f_trans_start_t) * 1e-6f;
			_vtol_schedule.flight_mode = MC_MODE;

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// Safety
			if (_local_pos->z > (- _params->vt_safe_alt)) {
				// initialise a front transition
				_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			}
			else
			{
				_vtol_schedule.flight_mode 	= MC_MODE;
				mavlink_log_critical(&mavlink_log_pub, "dangerous altitude");
			}
			
			_vtol_schedule.f_trans_start_t = hrt_absolute_time();

			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1: {
				bool airspeed_condition_satisfied = _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed;
				airspeed_condition_satisfied |= _params->airspeed_disabled;

				_vtol_schedule.fw_start = hrt_absolute_time();

				// check if we have reached airspeed  and the transition time is over the setpoint to switch to TRANSITION P2 mode
				//if (time_since_trans_start >= (_params->front_trans_duration + _params_tailsitter.front_trans_dur_p2)) {
				if (time_since_trans_start >= (_params->front_trans_duration)) {
					//_vtol_schedule.flight_mode = FW_MODE;
					//_lock_to_mc = 1;
					_vtol_schedule.flight_mode = TRANSITION_FRONT_P1; //Just for the wall suck

				}

				break;
			}

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;
		}
	}
	
	/* Safety altitude protection, stay at MC mode when trige for once */
	static bool alt_danger = false;
	if ((_local_pos->z < (- _params->vt_safe_alt)) || (alt_danger == true))
	{
		if((_vtol_schedule.flight_mode == FW_MODE) || (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1))
		{
			alt_danger             = true;
			mavlink_log_critical(&mavlink_log_pub, "dangerous altitude");
		}
		_vtol_schedule.flight_mode = MC_MODE;
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case TRANSITION_FRONT_P1:
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

float Tailsitter::calc_pitch_rot(float time_since_trans_start)
{
	float angle = 0.0;

	for (int i = 0; i <= (POINT_NUM - 1); i ++) {
		if (time_since_trans_start <= POINT_ACTION[0][i+1]) {
			angle = POINT_ACTION[1][i] + (time_since_trans_start - POINT_ACTION[0][i]) / (POINT_ACTION[0][i+1] - POINT_ACTION[0][i]) * (POINT_ACTION[1][i+1] - POINT_ACTION[1][i]);
			angle = DEG_TO_RAD(math::constrain(angle, 0.0f, 80.0f));
			break;
		}
		if (time_since_trans_start >= POINT_ACTION[0][POINT_NUM - 1]) {
			angle = DEG_TO_RAD(POINT_ACTION[1][POINT_NUM - 1]);
		}
	}

	return angle;
}

float Tailsitter::calib_tof_distance()
{
	_vtol_vehicle_status->lat_dist       = _pm3901_tof_data->tof_pos;
	_vtol_vehicle_status->lat_dist_calib = _pm3901_tof_data->tof_pos_calib;
	_vtol_vehicle_status->tof_vel        = _pm3901_tof_data->tof_vel;
	_vtol_vehicle_status->tof_vel_calib  = _pm3901_tof_data->tof_vel_calib;//positive: toward wall

	return _vtol_vehicle_status->lat_dist_calib;
}

float Tailsitter::track_path(float distance_to_wall, float vel_to_wall, float time_from_start)
{
	static int _integ_pos = 0.0f;
	static int _integ_vel = 0.0f;

	float min_pos = 0.22f;
	float T       = 3.0f;// overall time
	float a       = (_dist_sp - min_pos) / (T * T * T - 1.5f * T * T * T);// 0.132 is the last distance
	float b       = -1.5f * T * a;
	float pos_sp  = - a * powf(time_from_start, 3) - b * powf(time_from_start, 2) + _dist_sp;// pos=-a*t^3-b*t^2+c
	float vel_ff  = - 3.0f * a * powf(time_from_start, 2) - 2.0f * b * time_from_start;
	float roll_ff = - (- 6.0f * a * time_from_start - 2.0f * b) / 9.8f;

	if ((_vtol_vehicle_status->ticks_since_trans % 50) == 5) 
	{
		mavlink_log_critical(&mavlink_log_pub, "a:%.4f b:%.4f", (double)(a), (double)(b));
	}

	// pos_sp = math::constrain(pos_sp, min_pos, _dist_sp);

	if (pos_sp <= min_pos)
	{	
		pos_sp  = min_pos;
		vel_ff  = 0.0f;
		roll_ff = 0.0f;
	}

	_vtol_vehicle_status->dist_sp = pos_sp;

	float Kp_pos  = 3.0f;
	float Ki_pos  = 0.1f;
	float err_pos = distance_to_wall - pos_sp;
	float vel_sp  = Kp_pos * err_pos + _integ_pos - vel_ff;// positive: toward wall

	_integ_pos   += Ki_pos * err_pos * 0.01f;

	_vtol_vehicle_status->vel_sp = vel_sp;

	float Kp_vel  = 0.15f;
	float Ki_vel  = 0.015f;
	float err_vel = - vel_to_wall + vel_sp;
	float roll_sp = Kp_vel * err_vel + _integ_vel + roll_ff;
	_integ_vel   += Ki_vel * err_vel * 0.01f;

	roll_sp   = math::constrain(roll_sp, DEG_TO_RAD(-25.0f), DEG_TO_RAD(25.0f));

	return roll_sp;
}

void Tailsitter::update_transition_state()
{
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.f_trans_start_t) * 1e-6f;
	_vtol_vehicle_status->ticks_since_trans ++;
	_euler = matrix::Quatf(_v_att->q);
	float distance_to_wall = calib_tof_distance();
	
	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0, 0, -1));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition
			_q_trans_start = Eulerf(0.0f, _fw_virtual_att_sp->pitch_body, yaw_sp);

			// create time dependant pitch angle set point + 0.2 rad overlap over the switch value
			//_v_att_sp->pitch_body = _pitch_transition_start	- fabsf(PITCH_TRANSITION_FRONT_P1 - _pitch_transition_start) *
			//			time_since_trans_start / _params->front_trans_duration;
			//_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body, PITCH_TRANSITION_FRONT_P1,
			//					_pitch_transition_start);

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			_q_trans_start  = Quatf(_v_att->q);//Eulerf(0.0f, _mc_virtual_att_sp->pitch_body, _mc_virtual_att_sp->yaw_body);
			Vector3f x      = Dcmf(Quatf(_v_att->q)) * Vector3f(1, 0, 0);
			_trans_rot_axis = -x.cross(Vector3f(0, 0, -1));
			_trans_roll_axis = _trans_rot_axis.cross(Vector3f(0, 0, -1));
		}

		_q_trans_sp      = _q_trans_start;
		_alt_sp          = _local_pos->z;
		_dist_sp         = _pm3901_tof_data->tof_pos_calib;

		_mc_hover_thrust = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
	}

	_trans_end_thrust = _mc_hover_thrust;

	// tilt angle (zero if vehicle nose points up (hover))
	float tilt = acosf(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) - _q_trans_sp(2) * _q_trans_sp(
				   2) + _q_trans_sp(3) * _q_trans_sp(3));

	/*** Excute the movement on the forward transition period ***/
	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {
		switch (_params->vt_sweep_or_suck_type){
		case NO_SUCK:
		case TOP_WALL:
			{
				_q_trans_sp = Quatf(_v_att->q);
				//float acc_time = 0.7f;
				//if (time_since_trans_start <= acc_time)
				//{
				//	_trans_end_thrust = _mc_hover_thrust;
				//}
				//if (time_since_trans_start <= _params->front_trans_duration)
				//{
					_trans_end_thrust = _mc_hover_thrust *  _params->suck_thr_ratio;
				//}
				//else {
				//	_trans_end_thrust = _mc_hover_thrust *  _params->suck_thr_ratio;
				//}

				if ((_vtol_vehicle_status->ticks_since_trans % 50) == 5)
				{
					mavlink_log_critical(&mavlink_log_pub, "Start Thrust: %.4f Current: %.4f time: %.4f", (double)(_mc_hover_thrust), (double)(_trans_end_thrust), (double)(time_since_trans_start));
				}
			}	    	
			break;

	    case SIDE_WALL:
		    {
		    	float roll_sp = track_path(distance_to_wall, _pm3901_tof_data->tof_vel_calib, time_since_trans_start);
		    	_q_trans_sp   = Quatf(AxisAnglef(_trans_roll_axis, roll_sp)) * _q_trans_start;

		    	//_vtol_vehicle_status->dist_sp = _dist_sp;
		    	_vtol_vehicle_status->rollrot = roll_sp;
		    	if ((_vtol_vehicle_status->ticks_since_trans % 10) == 5)
				{
					mavlink_log_critical(&mavlink_log_pub, "Transition Start");
				}
			}
	    	break;
	    default:
	    	break;
		}

	}
	/*** Excute the movement on the suck top and suck wall period ***/
	else if (_vtol_schedule.flight_mode == TRANSITION_BACK)
	{
		_target_alt  = _local_pos->z;
		const float trans_pitch_rate = M_PI_2_F / _params->back_trans_duration;
		_vert_i_term = 0.0f;

		if (!flag_idle_mc) {
			flag_idle_mc = set_idle_mc();
		}

		if (tilt > 0.01f) {
			_trans_pitch_rot = time_since_trans_start * trans_pitch_rate;
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis, time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}
	/*** Reset the target altitude, integrator and pitch setpoint of the transition ***/
	else
	{
		_target_alt  = _local_pos->z; 
		_vert_i_term = 0.0f;
		_trans_pitch_rot = 0.0f;
	}

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_v_att_sp->roll_body = euler_sp.phi();
	_v_att_sp->pitch_body = euler_sp.theta();
	_v_att_sp->yaw_body = euler_sp.psi();
	_q_trans_sp.copyTo(_v_att_sp->q_d);

	_v_att_sp->q_d_valid = true;
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[2] = _thrust_transition;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

	// allow fw yawrate control via multirotor roll actuation. this is useful for vehicles
	// which don't have a rudder to coordinate turns
	if (_params->diff_thrust == 1) {
		_mc_roll_weight = 1.0f;
	}
}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	float time_since_fw_start = 0.0f;
	float smooth_fw_start = 0.0f;

	// parameters for sweep
	float time_since_sweep = 0.0f;
	float sweep_signal_phase = 0.0f;
	float sweep_signal = 0.0f;
	
	float sweep_min_frequency = 0.5f * 6.2831f;
	float sweep_max_frequency = 80.0f * 6.2831f ;
	float overall_time = 150.0f;

	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	switch (_vtol_mode) {
	case ROTARY_WING:
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL]     = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH]    = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW]      = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		if (_params->elevons_mc_lock) {
			_actuators_out_1->control[0] = 0;
			_actuators_out_1->control[1] = 0;

		} else {
			// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL]  = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];	//roll elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];	//pitch elevon
		}

		/* just for sweep input signal */
		if(_attc->is_sweep_requested()) {
			switch (_params->vt_sweep_or_suck_type){
			case NO_SWEEP:
				break;
			case PITCH_RATE:
				time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
				// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				//sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_or_suck_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] + sweep_signal;
				break;
		    case ROLL_RATE:
		    	time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
		    	// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				// sweep_signal_phase = sweep_min_frequency  * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_or_suck_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] + sweep_signal;
				break;
			case THRUST:
				time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
		    	// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				// sweep_signal_phase = sweep_min_frequency  * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_or_suck_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] + sweep_signal;
				break;
			}
		} else {
			/* record the start time */
			_vtol_schedule.sweep_start = hrt_absolute_time();
		}

		break;

	case FIXED_WING:
		// at the start of the fw mode, the control output of pitch is smoothed from the end of transition
		time_since_fw_start = (float)(hrt_absolute_time() - _vtol_schedule.fw_start) * 1e-6f;
		smooth_fw_start = math::constrain(time_since_fw_start / 0.5f, 0.0f, 1.0f);

		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + (1.0f - smooth_fw_start) * 0.0f + _params->fw_pitch_trim;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE]* smooth_fw_start + _trans_end_thrust * (1.0f - smooth_fw_start);

		break;

	case TRANSITION_TO_FW:
	case TRANSITION_TO_MC:
		// in transition engines are mixed by weight (BACK TRANSITION ONLY)
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL]     = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH]    = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW]      = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _trans_end_thrust;

		_vtol_schedule.ctrl_out_trans_end = _actuators_out_0->control[actuator_controls_s::INDEX_PITCH];
		break;
	}
}
