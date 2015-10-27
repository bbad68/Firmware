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
 * @file tiltframe.cpp
 *
 * @author Jonathan Lee     <jon@indrorobotics.com>
 *
 */

#include "tiltframe.h"
#include "vtol_att_control_main.h"

/**
 * TiltFrame object constructor, with default to MC Mode.
 */
TiltFrame::TiltFrame(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	/* Initialize system in MC mode.	*/
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.trans_start = 0;
	_vtol_schedule.trans_end = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_frame_angle = 0.0f;
	_angle_step = 0.0f;

	/* Link parameter handles. */
	_params_handles_tiltframe.mc_centre_angle =
		param_find("VT_MC_CENTRE_ANGLE");
	_params_handles_tiltframe.fw_centre_angle =
		param_find("VT_FW_CENTRE_ANGLE");
	_params_handles_tiltframe.max_angle_range =
		param_find("VT_MAX_ANGLE_RANGE");
	_params_handles_tiltframe.max_angle_step =
		param_find("VT_MAX_ANGLE_STEP");
}

/**
 * TiltFrame object destructor.
 */
TiltFrame::~TiltFrame()
{
	/* No explicit destructor required. */
}

/**
 * Update State Machine determining current Flight Mode.
 */
void TiltFrame::update_vtol_state()
{
	// TODO.
}

/**
 * Update control scheme for MC Mode.
 */
void TiltFrame::update_mc_state()
{
	// TODO.
}

/**
 * Update control scheme for FW Mode.
 */
void TiltFrame::update_fw_state()
{
	// TODO.
}

/**
 * Update control scheme for either Transition Mode.
 */
void TiltFrame::update_transition_state()
{
	// TODO.
}

/**
 * Update control scheme for external uORB topic subscribers.
 */
void TiltFrame::update_external_state()
{
    /* Unknown function, defined to match existing VTOL class definitions. */
}

/**
 * Set all three MC Weight variables simultaneously.
 */
void TiltFrame::set_mc_weights(float new_weight)
{
	/* Ensure "new_weight" is a valid value. */
	float temp_weight = math::constrain(new_weight, 0.0f, 1.0f);

	_mc_roll_weight = temp_weight;
	_mc_pitch_weight = temp_weight;
	_mc_yaw_weight = temp_weight;
}

/**
 * Prepare message to actuators with data from MC and FW attitude controllers.
 */
void TiltFrame::fill_actuator_outputs()
{
	// TODO.
}

int TiltFrame::parameters_update()
{
	/**
	 * Temporary variables for fetching parameter values.
	 */
	float v;

	/**
	 * Ensure MC centre angle is close to fully vertical.
	 */
	param_get(_params_handles_tiltframe.mc_centre_angle, &v);
	_params_tiltframe.mc_centre_angle = math::constrain(v, -5.0f, 5.0f);

	/**
	 * Ensure FW centre angle does not point motors downward and does not
	 * fall within region dominated by MC control scheme.
	 */
	param_get(_params_handles_tiltframe.fw_centre_angle, &v);
	_params_tiltframe.fw_centre_angle = math::constrain(v, 45.0f, 90.0f);

	/**
	 * Ensure frame range does not allow attempted movement beyond hardware
	 * limits of motor frame.
	 */
	float v_ceil = math::min(
		fabsf(_MAX_REV_ANGLE - _params_tiltframe.mc_centre_angle),
		fabsf(_MAX_FWD_ANGLE - _params_tiltframe.fw_centre_angle));
	param_get(_params_handles_tiltframe.max_angle_range, &v);
	_params_tiltframe.max_angle_range = math::constrain(v, 0.0f, v_ceil);

	/**
	 * Ensure controlable incremental motor frame rotation.
	 */
	param_get(_params_handles_tiltframe.max_angle_step, &v);
	_params_tiltframe.max_angle_step = math::constrain(v, 0.0f, 10.0f);

	return OK;
}
