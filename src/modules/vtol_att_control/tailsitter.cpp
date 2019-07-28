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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "vtol_att_control_main.h"
#include <systemlib/mavlink_log.h>
#include <math.h>
#include "tailsitter.h"

#ifndef M_PI
#define M_PI (3.14159265f)
#endif

#define RAD_TO_DEG(x) ((x) / 3.1416f * 180.0f)
#define DEG_TO_RAD(x) ((x) / 180.0f * 3.1416f)

#define ARSP_YAW_CTRL_DISABLE     (4.0f)	// airspeed at which we stop controlling yaw during a front transition
#define THROTTLE_TRANSITION_MAX   (0.25f)	// maximum added thrust above last value in transition
#define PITCH_TRANSITION_FRONT_P1 (-_params->front_trans_pitch_sp_p1)	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_BACK     (-0.25f)	// pitch angle to switch to MC
#define Max_Thrust_cmd 0.9f
#define	Min_Thrust_cmd 0.1f

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

	//mavlink_log_critical(&mavlink_log_pub, "sys_ident_cl_point:%.5f inttest:%d", (double)(_CL_Degree[19]), int(16.99f * 1));

}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	Eulerf euler = Quatf(_v_att->q);
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.f_trans_start_t) * 1e-6f;
	float time_since_b_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.b_trans_start_t) * 1e-6f;
	float pitch = euler.theta();

	if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
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

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_TRANSITION_BACK && time_since_b_trans_start >= 0.8f) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
		if ((_local_pos->z < (- _params->vt_safe_alt)) && !_vtol_schedule.vz_mission_finished){
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			_vtol_schedule.f_trans_start_t = hrt_absolute_time();
			_vtol_schedule.vz_mission_finished = true;
			_vtol_vehicle_status->vz_mission_finished = true;
		}
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1: {

				bool airspeed_condition_satisfied = _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed;
				airspeed_condition_satisfied |= _params->airspeed_disabled;

				_vtol_schedule.fw_start = hrt_absolute_time();

				// check if we have reached airspeed  and the transition time is over the setpoint to switch to TRANSITION P2 mode
				/*
				if ((airspeed_condition_satisfied && (time_since_trans_start >= (_params->front_trans_duration + _params_tailsitter.front_trans_dur_p2))) || can_transition_on_ground()) {
					//_vtol_schedule.flight_mode = FW_MODE;
					_vtol_schedule.flight_mode = MC_MODE;
				}
				*/
				if (time_since_trans_start >= (_params->front_trans_duration + _params_tailsitter.front_trans_dur_p2)){
					_vtol_schedule.flight_mode = MC_MODE;
				}
				break;
			}

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	}
	
	/* Safety altitude protection, stay at MC mode when trige for once */
	static bool alt_danger = false;
	if ((_local_pos->z > (- _params->vt_safe_alt)) || (alt_danger == true))
	{
		if((_vtol_schedule.flight_mode == FW_MODE) || (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1))
		{
			alt_danger             = true;
			//mavlink_log_critical(&mavlink_log_pub, "dangerous altitude");
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
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

float Tailsitter::get_CL(float aoa)
{
	float aoa_degree = RAD_TO_DEG(aoa);
	int   aoa_int    = 0;
	float CL         = 0.0f;

	if ((fabsf(aoa_degree) <= 89.99f) && (fabsf(aoa_degree) >= 0.01f)) 
	{
		aoa_int = aoa_degree / 1;
		CL      = _CL_Degree[aoa_int] + (aoa_degree - aoa_int) * (_CL_Degree[aoa_int + 1] - _CL_Degree[aoa_int]);
	}
	else
	{
		CL = 0.0f;
	}

	return CL;
}

/***
 *	calculate the thrust feedforward cmd based on the vertical acceleration cmd, horizontal velocity, pitch angle and ang-of-attack
 *	@input: vertical acceleration cmd (up is positive)
 *			horizontal velocity
 *			pitch angle
 *			ang-of-attack
 *	@output: thrust feedforward cmd
 ***/
float Tailsitter::thr_from_acc_cmd(float vert_acc_cmd, float airspeed, float pitch_ang, float vz)
{
	float bx_acc_cmd      = 0.0f;
	float bx_acc_err      = 0.0f;
	float bx_acc_err_i    = 0.0f;

	/* bx_acc_kp and bx_acc_ki are from loopshaping */
	float bx_acc_kp       = 0.006f;
	float bx_acc_ki       = 0.003f;

	float thrust_cmd      = 0.0f;
	float cos_pitch       = 0.0f;

	/* calculate the aerodynamic lift force */
	//float CL_temp           = 0.0f;
	//float lift_weight_ratio = 0.0f;
	float ang_of_attack     = 0.0f;
	//float dyn_pressure      = 0.5f * 1.237f * airspeed * airspeed;

	float ang_of_vel    = atan2f(vz, airspeed) * (math::constrain(vz * vz / (5.0f * 5.0f), 0.0f, 1.0f));

	pitch_ang           = math::constrain(pitch_ang,     DEG_TO_RAD(0.001f), DEG_TO_RAD(89.99f));
	ang_of_attack       = math::constrain(ang_of_vel, DEG_TO_RAD(-20.0f), DEG_TO_RAD(20.0f)) + DEG_TO_RAD(100.0f) - pitch_ang;
	ang_of_attack       = math::constrain(ang_of_attack, DEG_TO_RAD(0.001f), DEG_TO_RAD(89.99f));
	
	_vtol_vehicle_status->aoa = ang_of_attack;

	if ((fabsf(ang_of_attack) < DEG_TO_RAD(89.999f)) && (fabsf(ang_of_attack) >= DEG_TO_RAD(0.001f)))
	{
		/***
		CL_temp           = get_CL(ang_of_attack);
		lift_weight_ratio = dyn_pressure * 1.0f * CL_temp * 0.5f/ (1.68f * 9.8f);
		thrust_cmd        = (-_mc_hover_thrust - lift_weight_ratio * (-_mc_hover_thrust) + vert_acc_cmd / 9.8f * (-_mc_hover_thrust)) / cosf(pitch_ang);
		 ***/
		cos_pitch     = math::constrain(cosf(pitch_ang), 0.2f, 1.0f);
		bx_acc_cmd    = (9.8f + _sensor_acc->z * sinf(pitch_ang) - vert_acc_cmd) / cos_pitch;
		bx_acc_cmd    = math::constrain(bx_acc_cmd, -2.0f * 9.8f, 2.0f * 9.8f);
		bx_acc_err    = bx_acc_cmd - _sensor_acc->x;
		bx_acc_err_i  = _vtol_vehicle_status->bx_acc_i + bx_acc_ki * bx_acc_err * 0.004f;
		thrust_cmd    = bx_acc_cmd / 9.8f * (-_mc_hover_thrust) + bx_acc_err * bx_acc_kp + bx_acc_err_i;
	}
	else
	{
		//lift_weight_ratio = 0.0f;
		thrust_cmd        = -_mc_hover_thrust;
		bx_acc_err_i      = 0.0f;
	}

	_vtol_vehicle_status->pitch_ang  = pitch_ang;
	_vtol_vehicle_status->bx_acc_cmd = bx_acc_cmd;
	_vtol_vehicle_status->bx_acc_e   = bx_acc_err;
	_vtol_vehicle_status->bx_acc_i   = bx_acc_err_i;

	//thrust_cmd = math::constrain(thrust_cmd, 0.1f, 0.9f);
	static int ii = 0;
	ii++;
	if ((ii % 10) == 0) 
	{
		//mavlink_log_critical(&mavlink_log_pub, "airsp:%.2f lift:%.2f aoa:%.3f", (double)(airspeed), (double)(lift_weight_ratio), (double)(ang_of_attack));
	}

	return thrust_cmd;
}

float ILC_in(float time_since_trans_start)
{
	if ((time_since_trans_start > 0.0001f) && (time_since_trans_start < 6.0f)) {
		int   index_left = int(time_since_trans_start * 25);
		float input      = U_ILC_INPUT[index_left] + (time_since_trans_start * 25.0f - index_left) * (U_ILC_INPUT[index_left + 1] - U_ILC_INPUT[index_left]);

		return input;
	}
	else {
		return 0.0f;
	}
}

/***
 *	calculate the thrust cmd using feedforward and feedback controller
 *	@input: 
 *	@output:
 ***/
float Tailsitter::control_altitude(float time_since_trans_start, float alt_cmd)
{
	float alt_kp         = 5.0f;
	float vel_kp         = 3.0f;

	/* calculate the euler angle from quatnion(follow the Pitch-Roll-Yaw) */
	matrix::EulerFromQuatf euler = matrix::Quatf(_v_att->q);

	/* calculate the feedforward thrust cmd */
	float pitch          = - euler.theta(); // theta is minus zero

	//float ang_of_attack  = DEG_TO_RAD(96.0f) - pitch; //install pitch angle is 6 degree
	float horiz_vel      = sqrtf((_local_pos->vx * _local_pos->vx) + (_local_pos->vy * _local_pos->vy));
	float ILC_input      = ILC_in(time_since_trans_start);
	float vert_acc_cmd   = (alt_cmd - _local_pos->z) * alt_kp + -_local_pos->vz * vel_kp ;//+ ILC_input * 9.8f / (-_mc_hover_thrust);
	vert_acc_cmd         = math::constrain(vert_acc_cmd, -2.0f*9.8f, 2.0f*9.8f);
	float thrust_cmd     = math::constrain(thr_from_acc_cmd(vert_acc_cmd, horiz_vel, pitch, _local_pos->vz), 0.20f, 0.85f);

	_vtol_vehicle_status->ilc_input 	= ILC_input;
	_vtol_vehicle_status->vert_acc_cmd      = vert_acc_cmd;
	_vtol_vehicle_status->thrust_cmd        = thrust_cmd;
	_vtol_vehicle_status->ticks_since_trans ++;

	if ((_vtol_vehicle_status->ticks_since_trans % 10) == 5) 
	{
		//mavlink_log_critical(&mavlink_log_pub, "vert_acc_cmd:%.2f thrust_cmd:%.2f", (double)(vert_acc_cmd), (double)(thrust_cmd));
	}
	
	return (-1.0f * thrust_cmd);
}

float calc_pitch_rot(float time_since_trans_start) {
	float angle = 0.0;

	for (int i = 0; i <= (POINT_NUM - 1); i ++) {
		if (time_since_trans_start <= POINT_ACTION[0][i+1]) {
			angle = POINT_ACTION[1][i] + (time_since_trans_start - POINT_ACTION[0][i]) / (POINT_ACTION[0][i+1] - POINT_ACTION[0][i]) * (POINT_ACTION[1][i+1] - POINT_ACTION[1][i]);
			angle = DEG_TO_RAD(math::constrain(angle, 0.0f, 90.0f));
			break;
		}
		if (time_since_trans_start >= POINT_ACTION[0][POINT_NUM - 1]) {
			angle = DEG_TO_RAD(POINT_ACTION[1][POINT_NUM - 1]);
		}
	}

	return angle;
}


/* 
*  Calculate the vz command according to the predesigned trajectory  
*  Vz increase from a certain minimum speed Min_Speed to a maximum speed Max_Speed.
*  Every 1m/s there will be a fixed speed flight interval lasting for KeepTime secs.
*  AccTime secs will be cost to accelerate the vehicle to increase 1m/s
*  To smooth the step trajectory, the acceleration interval will be designed from a 
*  sigmoid function.
*/
float Tailsitter::calc_vz_cmd(float time_since_trans_start){
	int vz_cmd_index;
	float vz_change_period, current_vz_cmd, time_in_perioid;
	float k_sigmoid = 20 / _params->vt_vz_acctime, time_in_sigmoid = 0, sigmoid_value = 0;

	vz_change_period = _params->vt_vz_acctime + _params->vt_vz_keeptime;
	vz_cmd_index = floor(time_since_trans_start / vz_change_period);
	time_in_perioid = time_since_trans_start - vz_cmd_index * vz_change_period;
	current_vz_cmd = _params->vt_vz_minspeed + vz_cmd_index * 1.0f;

	if (time_in_perioid > _params->vt_vz_keeptime ){

		time_in_sigmoid = time_in_perioid - _params->vt_vz_keeptime - 0.5f * _params->vt_vz_acctime;
		sigmoid_value = 1/(1 + exp(-k_sigmoid * time_in_sigmoid));
		current_vz_cmd += sigmoid_value;
	}

	if (current_vz_cmd > _params->vt_vz_maxspeed) {
		current_vz_cmd = 0;
	}

	return -current_vz_cmd;
}
/*
*  A PID Control will be applied to follow the vz command
*  Saturation limitation for integrate controller is applied
*  
*
*/

float Tailsitter::control_vertical_speed(float vz, float vz_cmd){
	float thrust_cmd = 0;
	float error = 0;
	float P_output, I_output, D_output, PID_output;
	float now, dt;
	float Kp, Ki, Kd;
	Kp = _params->vt_vz_control_kp;
	Ki = _params->vt_vz_control_ki;
	Kd = _params->vt_vz_control_kd;
	now = float(hrt_absolute_time()) * 1e-6f;
	dt = now - _VZ_PID_Control.last_run;
	_VZ_PID_Control.last_run = now;
	vz_cmd = - vz_cmd;
	vz = -vz;
	error = vz_cmd - vz;

	// Integral Saturation 
	if (_VZ_PID_Control.is_saturated){
		Ki = 0;
	};
	mavlink_log_critical(&mavlink_log_pub, "vz_cmd is :%.5f  vz is: %.5f", (double)(vz_cmd), (double)(vz));
	// PID Control
	P_output = Kp * error;
	D_output = (error - _VZ_PID_Control.last_D_state) * Kd / dt;
	I_output = _VZ_PID_Control.last_I_state + Ki * error * dt;
	_VZ_PID_Control.last_D_state = error;
	_VZ_PID_Control.last_I_state = I_output;
	PID_output = P_output + I_output + D_output;
	
	thrust_cmd = -PID_output + _mc_hover_thrust;
	
	_VZ_PID_Control.is_saturated = false;
	if (-thrust_cmd >= Max_Thrust_cmd){
		_VZ_PID_Control.is_saturated = true;
		thrust_cmd = -Max_Thrust_cmd;
	} else if (-thrust_cmd < Min_Thrust_cmd){
		_VZ_PID_Control.is_saturated = true;
		thrust_cmd = -Min_Thrust_cmd;
	}

	mavlink_log_critical(&mavlink_log_pub, "thrust cmd is :%.5f", (double)(thrust_cmd));

	return thrust_cmd;
}

void Tailsitter::calc_q_trans_sp(){
	float lateral_dist, longitudinal_dist;
	float lateral_v, longitudinal_v;
	float rollrot, pitchrot;
	float Kp,Kvp;
	float P_output, Pv_output;
	float delt_x, delt_y;
	float vx, vy;



	delt_x = _local_pos->x - _trans_start_x;
	delt_y = _local_pos->y - _trans_start_y;
	vx = _local_pos->vx;
	vy = _local_pos->vy;
	lateral_dist = sqrtf(delt_x * delt_x + delt_y * delt_y) * sinf(atan2f(delt_y, delt_x) - _mc_virtual_att_sp->yaw_body);
	longitudinal_dist = sqrtf(delt_x * delt_x + delt_y * delt_y) * cosf((atan2f(delt_y, delt_x) - _mc_virtual_att_sp->yaw_body));
	lateral_v = sqrtf(vx * vx + vy * vy) * sinf(atan2f(vy, vx) - _mc_virtual_att_sp-> yaw_body);
	longitudinal_v = sqrtf(vx * vx + vy * vy) * cosf(atan2f(vy, vx) - _mc_virtual_att_sp-> yaw_body);

	// PI controller of lateral dist
	Kp = _params->vt_y_dist_kp;
	Kvp = _params->vt_vy_kp;

	P_output = Kp * lateral_dist;
	Pv_output = Kvp * lateral_v;
	rollrot = P_output + Pv_output;
	_trans_roll_rot  = math::constrain(rollrot, -0.15f, 0.15f);

	// PI controller of longitudinal dist
	Kp = _params->vt_x_dist_kp;
	Kvp = _params->vt_vx_kp;

	P_output = Kp * longitudinal_dist;
	Pv_output = Kvp * longitudinal_v;
	pitchrot = P_output + Pv_output;	
	_trans_pitch_rot = math::constrain(pitchrot, -0.15f,0.15f);


	_vtol_vehicle_status->longitudinal_dist = longitudinal_dist;
	_vtol_vehicle_status->pitchrot 		= _trans_pitch_rot;
	_vtol_vehicle_status->rollrot           = _trans_roll_rot;
	_vtol_vehicle_status->lat_dist          = lateral_dist;
	_vtol_vehicle_status->longitudinal_v 	= longitudinal_v;
	_vtol_vehicle_status->lateral_v 	= lateral_v;

	_q_trans_sp = Quatf(AxisAnglef(_trans_roll_axis, _trans_roll_rot))*Quatf(AxisAnglef(_trans_rot_axis, _trans_pitch_rot)) * _q_trans_start;

}

void Tailsitter::update_transition_state()
{
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.f_trans_start_t) * 1e-6f;
	float vz_cmd;


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
			_q_trans_start  = Eulerf(0.0f, 0.0f, _mc_virtual_att_sp->yaw_body);
			Vector3f x      = Dcmf(Quatf(_v_att->q)) * Vector3f(1, 0, 0);
			_trans_rot_axis = -x.cross(Vector3f(0, 0, -1));
			_trans_roll_axis  = _trans_rot_axis.cross(Vector3f(0, 0, -1));
		}

		_q_trans_sp      = _q_trans_start;
		_alt_sp          = _local_pos->z;

		_mc_hover_thrust = _v_att_sp->thrust_body[2];
	}

	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	// tilt angle (zero if vehicle nose points up (hover))
	float tilt = acosf(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) - _q_trans_sp(2) * _q_trans_sp(
				   2) + _q_trans_sp(3) * _q_trans_sp(3));

	// calculate the pitch setpoint
	// For system identification experiment of quadrotor thrust test,
	// the pitch command is always set to 0 while the thrust_cmd is 
	// calculate according to the vertical speed vz by a PID controller.
	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {

		// const float trans_pitch_rate = M_PI_2_F / _params->front_trans_duration;
		//_trans_pitch_rot = calc_pitch_rot(time_since_trans_start);
		_trans_pitch_rot = 0.0f;

		// Calculate the velocity setpoint
		vz_cmd = calc_vz_cmd(time_since_trans_start);
		_vtol_vehicle_status->vz_cmd = vz_cmd;


		/* lateral and longitudinal control */

		calc_q_trans_sp();

		/* calculate the thrust cmd to control altitude*/
		/* _v_att_sp->thrust_body[2] = control_altitude(time_since_trans_start, _alt_sp);*/

		/* calculate the thrust cmd to control the vertical speed*/
		_v_att_sp -> thrust_body[2] = control_vertical_speed(_local_pos->vz, vz_cmd);

		/* save the thrust value at the end of the transition */
		_trans_end_thrust = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
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
	} else {
		_target_alt  = _local_pos->z;
		_vert_i_term = 0.0f;
		_vtol_vehicle_status->bx_acc_i = 0.0f;
		_trans_pitch_rot = 0.0f;
	}

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

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
	float time_since_sweep = 0.0f;
	float sweep_signal_phase = 0.0f;
	float sweep_signal = 0.0f;
	float smooth_fw_start = 0.0f;
	float smooth_pr_start = 0.0f;
	float sweep_min_frequency = 0.5f * 6.2831f;
	float sweep_max_frequency = 80.0f * 6.2831f ;
	float overall_time = 150.0f;
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	switch (_vtol_mode) {
	case ROTARY_WING:
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		if (_params->elevons_mc_lock) {
			_actuators_out_1->control[0] = 0;
			_actuators_out_1->control[1] = 0;

		} else {
			// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_YAW];	//roll elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];	//pitch elevon
		}

		/* just for sweep input signal */
		if(_attc->is_sweep_requested()) {
			switch (_params->vt_sweep_type){
			case NO_SWEEP:
				break;
			case PITCH_RATE:
				time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
				// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				//sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] + sweep_signal;
				break;
		    	case ROLL_RATE:
		    		time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
		    		// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				// sweep_signal_phase = sweep_min_frequency  * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] + sweep_signal;
				break;
			case THRUST:
				time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
		    	// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				// sweep_signal_phase = sweep_min_frequency  * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_amp) * sinf(sweep_signal_phase);
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
		smooth_fw_start = math::constrain(time_since_fw_start / 0.3f, 0.0f, 1.0f);
		smooth_pr_start = math::constrain(time_since_fw_start / 1.5f, 0.0f, 1.0f);

		if (time_since_fw_start <= 0.05f)
		{
			_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE] * smooth_fw_start + 0.70f * (1.0f - smooth_fw_start);
		}
		else if (time_since_fw_start <= 1.5f)
		{
			_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] * smooth_pr_start
				+ (_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) * (1.0f - smooth_pr_start);
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * smooth_pr_start
				+ _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * (1.0f - smooth_pr_start);
			_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
				-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]  * smooth_pr_start
				+ _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * (1.0f - smooth_pr_start);


		} else{
			_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim;
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];
			_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
				-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
		}

		
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE] * smooth_fw_start + 0.70f * (1.0f - smooth_fw_start);

		#if 0
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim;	// pitch elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];	// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];	// throttle
		#endif

		break;

	case TRANSITION_TO_FW:
	case TRANSITION_TO_MC:
		// in transition engines are mixed by weight (BACK TRANSITION ONLY)
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		_vtol_schedule.ctrl_out_trans_end = _actuators_out_0->control[actuator_controls_s::INDEX_PITCH];

		// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]
				* (1 - _mc_yaw_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		// **LATER** + (_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) *(1 - _mc_pitch_weight);
		//_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
		//	_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
		break;
	}
}
