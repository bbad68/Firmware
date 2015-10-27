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
	parameters_update();

	/**
	 * The state of the VTOL is determined by the position of a two position
	 * switch, which sets the desired Flight Mode, and the angle of the motor
	 * frame relative to the target forward speed (set via either pilot or
	 * software control).
	 */
	if (_attc->is_fixed_wing_requested() == true)
	{
		/**
		 * Switch is currently set to Fixed Wing Mode. Now check current
		 * state to determine required transition action(s).
		 */
		switch (_vtol_schedule.flight_mode)
		{
		case MC_MODE:
			/* Begin transition to FW Mode. */
	        _vtol_schedule.flight_mode = TRANSITION_TO_FW;
	        _vtol_schedule.trans_start = hrt_absolute_time();
	        break;
	    case FW_MODE:
	        /* Maintain flight in FW Mode. */
	        if ((_frame_angle <
					_params_tiltframe.fw_centre_angle -
					_params_tiltframe.max_angle_range) ||
	            (_frame_angle >
	                _params_tiltframe.fw_centre_angle +
	                _params_tiltframe.max_angle_range))
	        {
	            /**
	             * Motor frame outside bounds of FW Mode range, revert to
	             * corresponding Transition mode.
	             */
	            _vtol_schedule.flight_mode = TRANSITION_TO_FW;
	        }
	        break;
	    case TRANSITION_TO_FW:
	        /* Continue the Transition to FW Mode. */
	        if ((_frame_angle >=
	                _params_tiltframe.fw_centre_angle) &&
	        	(_frame_angle <=
	                _params_tiltframe.fw_centre_angle +
	                _params_tiltframe.max_angle_range))
	        {
	            /* Transition complete, frame within FW Mode range. */
	            _vtol_schedule.flight_mode = FW_MODE;
	            _vtol_schedule.trans_end = hrt_absolute_time();
	        }
	        break;
	    case TRANSITION_TO_MC:
	        /**
	         * Switch was flipped during Transition to MC. Stop previous
	         * transition attempt and begin Transition to FW.
	         */
	        _vtol_schedule.flight_mode = TRANSITION_TO_FW;
	        _vtol_schedule.trans_start = hrt_absolute_time();
	        break;
	    }
	}
	else if (_attc->is_fixed_wing_requested() == false)
	{
	    /**
	     * Switch is currently set to Multi-Copter Mode. Now check current
	     * state to determine required transition action(s).
		 */
        switch (_vtol_schedule.flight_mode)
        {
        case MC_MODE:
            /* Maintain flight in MC Mode. */
            if ((_frame_angle <
                    _params_tiltframe.mc_centre_angle -
                    _params_tiltframe.max_angle_range) ||
                (_frame_angle >
                    _params_tiltframe.mc_centre_angle +
                    _params_tiltframe.max_angle_range))
            {
                /**
                 * Motor frame outside bounds of MC Mode range, revert to
                 * corresponding Transition mode.
                 */
                _vtol_schedule.flight_mode = TRANSITION_TO_MC;
            }
            break;
        case FW_MODE:
            /* Begin transition to MC Mode. */
            _vtol_schedule.flight_mode = TRANSITION_TO_MC;
            _vtol_schedule.trans_start = hrt_absolute_time();
            break;
        case TRANSITION_TO_FW:
            /**
             * Switch was flipped during Transition to FW. Stop previous
             * transition attempt and begin Transition to MC.
             */
            _vtol_schedule.flight_mode = TRANSITION_TO_MC;
            _vtol_schedule.trans_start = hrt_absolute_time();
            break;
        case TRANSITION_TO_MC:
            /* Continue the Transition to MC Mode. */
            if ((_frame_angle >=
                    _params_tiltframe.mc_centre_angle -
                    _params_tiltframe.max_angle_range) &&
                (_frame_angle <=
                	_params_tiltframe.mc_centre_angle))
            {
                /* Transition complete, frame within MC Mode range. */
                _vtol_schedule.flight_mode = MC_MODE;
                _vtol_schedule.trans_end = hrt_absolute_time();
            }
            break;
        }
    }

    /* Map specific control phases to simple control modes. */
    switch (_vtol_schedule.flight_mode)
    {
    case MC_MODE:
        _vtol_mode = ROTARY_WING;
        break;
    case FW_MODE:
        _vtol_mode = FIXED_WING;
        break;
    case TRANSITION_TO_FW:
    case TRANSITION_TO_MC:
        _vtol_mode = TRANSITION;
        break;
    }
}

/**
 * Update control scheme for MC Mode.
 */
void TiltFrame::update_mc_state()
{
	if (_vtol_schedule.flight_mode == MC_MODE)
	{
		/**
		 * Change motor frame angle to counter pitch input from pilot.
		 * Intended operation is to have fuselage remain level while the
		 * aircraft moves forward with a tilted frame.
		 */
		_angle_diff =
			(_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] *
			_params_tiltframe.max_angle_range) - _frame_angle;
		_angle_step = math::constrain(
			_angle_diff,
			-(_params_tiltframe.max_angle_step),
			_params_tiltframe.max_angle_step);
		_frame_angle += _angle_step;

		/* Explicitly set MC Control Weights to One (full MC Control). */
		set_mc_weights(1.0f);
	}
}

/**
 * Update control scheme for FW Mode.
 */
void TiltFrame::update_fw_state()
{
	if (_vtol_schedule.flight_mode == FW_MODE)
    {
        /**
         * Change motor frame angle based on desired forward velocity.
         * Intended operation is to have frame tilt forward in proportion
         * to throttle, in order to steadily increase fraction of lift
         * generated by the wings.
         */
        _angle_diff =
            (_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE] *
            _params_tiltframe.max_angle_range) - _frame_angle;
        _angle_step = math::constrain(
            _angle_diff,
            -(_params_tiltframe.max_angle_step),
            _params_tiltframe.max_angle_step);
        _frame_angle += _angle_step;

        /* Explicitly set MC Control Weights to Zero (full FW Control). */
        set_mc_weights(0.0f);
    }
}

/**
 * Update control scheme for either Transition Mode.
 */
void TiltFrame::update_transition_state()
{
	if (_vtol_mode == TRANSITION)
    {
        /* Change motor frame angle incrementally during transition states. */
        if (_vtol_schedule.flight_mode == TRANSITION_TO_FW)
        {
            _angle_diff = _params_tiltframe.fw_centre_angle - _frame_angle;
        }
        else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC)
        {
            _angle_diff = _params_tiltframe.mc_centre_angle - _frame_angle;
        }
        _angle_step = math::constrain(
            _angle_diff,
            -(_params_tiltframe.max_angle_step),
            _params_tiltframe.max_angle_step);
        _frame_angle += _angle_step;

        /* Update control weighting based on new frame angle. */
        set_mc_weights(cos(_frame_angle * ANG_TO_RAD));
    }
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
	/**
     * The various Control Groups and Actuator Indices are defined at:
     * http://pixhawk.com/dev/mixing#control_groups
     */

    /* Multi-Copter Controls. */
    _actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
        _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] *
        _mc_roll_weight;
    _actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
        _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] *
        _mc_pitch_weight;
    _actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
        _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
        _mc_yaw_weight;

    /* Fixed Wing Controls. */
    _actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
        _actuators_fw_in->control[actuator_controls_s::INDEX_ROLL] *
        (1 - _mc_roll_weight);
    _actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
        (_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] +
        _params->fw_pitch_trim) * (1 - _mc_pitch_weight);
    _actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
        _actuators_fw_in->control[actuator_controls_s::INDEX_YAW] *
        (1 - _mc_yaw_weight);

    /* Throttle Control. */
    if (_vtol_schedule.flight_mode == FW_MODE)
    {
        _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
            _actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
    }
    else
    {
        /* Default to MC Throttle Control for remaining Flight Modes. */
        _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
            _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
    }

    /**
     * Motor Frame Control.
     * Index "4" in Control Group 1 is defined for Aux0 Pin.
     */
    _actuators_out_1->control[4] =
        math::constrain(_frame_angle, _MAX_REV_ANGLE, _MAX_FWD_ANGLE) /
        _MAX_FWD_ANGLE;
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
