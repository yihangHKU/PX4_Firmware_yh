/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTasks_generated.hpp
 *
 * Generated Header to list all required flight tasks
 *
 * @author Christoph Tobler <christoph@px4.io>
 */

 #pragma once

// include all required headers
#include "FlightTasks.hpp"
#include "FlightTaskManualAltitude.hpp"
#include "FlightTaskManualAltitudeSmooth.hpp"
#include "FlightTaskManualPosition.hpp"
#include "FlightTaskManualPositionSmooth.hpp"
#include "FlightTaskManualPositionSmoothVel.hpp"
#include "FlightTaskSport.hpp"
#include "FlightTaskAutoLine.hpp"
#include "FlightTaskAutoLineSmoothVel.hpp"
#include "FlightTaskAutoFollowMe.hpp"
#include "FlightTaskOffboard.hpp"
#include "FlightTaskFailsafe.hpp"
#include "FlightTaskTransition.hpp"
#include "FlightTaskOrbit.hpp"

enum class FlightTaskIndex : int {
    None = -1,
    ManualAltitude,
    ManualAltitudeSmooth,
    ManualPosition,
    ManualPositionSmooth,
    ManualPositionSmoothVel,
    Sport,
    AutoLine,
    AutoLineSmoothVel,
    AutoFollowMe,
    Offboard,
    Failsafe,
    Transition,
    Orbit,

    Count // number of tasks
};

union TaskUnion {
    TaskUnion() {}
    ~TaskUnion() {}

    FlightTaskManualAltitude manualAltitude;
    FlightTaskManualAltitudeSmooth manualAltitudeSmooth;
    FlightTaskManualPosition manualPosition;
    FlightTaskManualPositionSmooth manualPositionSmooth;
    FlightTaskManualPositionSmoothVel manualPositionSmoothVel;
    FlightTaskSport sport;
    FlightTaskAutoLine autoLine;
    FlightTaskAutoLineSmoothVel autoLineSmoothVel;
    FlightTaskAutoFollowMe autoFollowMe;
    FlightTaskOffboard offboard;
    FlightTaskFailsafe failsafe;
    FlightTaskTransition transition;
    FlightTaskOrbit orbit;
};
