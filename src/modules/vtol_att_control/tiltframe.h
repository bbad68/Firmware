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
 * @file tiltframe.h
 *
 * @author Jonathan Lee 		<jon@indrorobotics.com>
 *
 */

#ifndef TILTFRAME_H
#define TILTFRAME_H

#ifndef PI
#define PI 3.1415926535f
#endif

#ifndef ANG_TO_RAD
#define ANG_TO_RAD (PI / 180.0f)
#endif

#include <drivers/drv_hrt.h>
#include <math.h>

#include "vtol_type.h"

class TiltFrame : public VtolType
{

public:

	TiltFrame(VtolAttitudeControl * _att_controller);
	~TiltFrame();

	void update_vtol_state();
	void update_mc_state();
	void update_fw_state();
	void update_transition_state();
	void update_external_state();

private:

	/**
	 * Member variables.
	 */
	struct
	{
		/* Motor frame centre in MC Mode (hardware limited) [deg]. */
		float mc_centre_angle;
		/* Motor frame centre in FW Mode (hardware limited) [deg]. */
		float fw_centre_angle;
		/* Max frame movement away from centre (hardware limited) [deg]. */
		float max_angle_range;
		/* Max increment for motor frame angle from one poll loop [deg]. */
		float max_angle_step;
	} _params_tiltframe;

	struct
	{
		param_t mc_centre_angle;
		param_t fw_centre_angle;
		param_t max_angle_range;
		param_t max_angle_step;
	} _params_handles_tiltframe;

	enum vtol_mode
	{
		/* Multicopter Mode (Default). */
		MC_MODE = 0,
		/* Forward Transition. */
		TRANSITION_TO_FW,
		/* Reverse Transition. */
		TRANSITION_TO_MC,
		/* Fixed Wing Mode. */
		FW_MODE
	};

	struct
	{
		/* Current VTOL Flight Mode.	*/
		vtol_mode flight_mode;
		/* Absolute time at which most recent transition started. */
		hrt_abstime trans_start;
		/* Absolute time at which most recent transition ended. */
		hrt_abstime trans_end;
	} _vtol_schedule;

	/* Current angle of motor frame [deg]. */
	float _frame_angle;
	/* Difference between current and desired motor frame angles [deg]. */
	float _angle_diff;
	/* Change in motor frame angle to occur during next cycle [deg]. */
	float _angle_step;

	/* Max reverse angle motor frame can tilt (hardware defined) [deg]. */
	const float _MAX_REV_ANGLE = -30.0;
	/* Max forward angle motor frame can tilt (hardware defined) [deg]. */
	const float _MAX_FWD_ANGLE = 90.0;

	/**
	 * Member functions.
	 */
	void set_mc_weights(float new_weight);

	void fill_actuator_outputs();
	int parameters_update();
};
#endif
