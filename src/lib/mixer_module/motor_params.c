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
 * @file motor_params.c
 *
 * Parameters for motors.
 *
 */


/**
 * Minimum motor rise time (slew rate limit).
 *
 * Minimum time allowed for the motor input signal to pass through
 * a range of 1000 PWM units. A value x means that the motor signal
 * can only go from 1000 to 2000 PWM in minimum x seconds.
 *
 * Zero means that slew rate limiting is disabled.
 *
 * @min 0.0
 * @unit s/(1000*PWM)
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(MOT_SLEW_MAX, 0.0f);

/**
 * Thrust to motor control signal model parameter
 *
 * Parameter used to model the nonlinear relationship between
 * motor control signal (e.g. PWM) and static thrust.
 *
 * The model is: rel_thrust = factor * rel_signal^2 + (1-factor) * rel_signal,
 * where rel_thrust is the normalized thrust between 0 and 1, and
 * rel_signal is the relative motor control signal between 0 and 1.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(THR_MDL_FAC, 0.0f);

// /**
//  * Custom debug Parameter
//  *
//  * @group AUTO_PREFLIGHT_CHECK
//  * @value 2 - Mode 1: 2 Sine Wave
//  * @value 3 - Mode 2: 3 Sine Wave
//  * @value 4 - Mode 3: 4 Sine Wave
//  * @value 5 - Mode 4: 5 Sine Wave
//  * @min 2
//  * @max 5
//  */
// PARAM_DEFINE_INT32(A_PFL_SV_LOOP, 2);


// /**
//  * Custom INT for AUTO_PREFLIGHT_CHECK Boolean
//  *
//  * 0 for Disable
//  * 1 for Enable
//  *
//  * @boolean
//  * @group AUTO_PREFLIGHT_CHECK
//  */
// PARAM_DEFINE_INT32(A_PFL_EN, 0);

// // /**
// //  *
// //  * Variations of Printing Behavior
// //  *
// //  * @group AUTO_PREFLIGHT_CHECK
// //  * @value 0 - Not Print
// //  * @value 1 - Print Case1
// //  * @value 2 - Print Case2
// //  * @value 3 - Print Case3
// //  * @min 0
// //  * @max 3
// // */
// // PARAM_DEFINE_INT32(AUTO_PFL_MT_LOOP, 0);

// /**
//  * Custom debug Parameter
//  *
//  * Default Motor Spin time is 1.5 Sec. 
//  *
//  * @min 0.50
//  * @max 3.00
//  * @decimal 5
//  * @increment 0.001
//  * @group AUTO_PREFLIGHT_CHECK
//  */
// PARAM_DEFINE_FLOAT(A_PFL_MT_TIME, 1.5f);

// /**
//  *
//  * Actuator Testing Case
//  *
//  * @group AUTO_PREFLIGHT_CHECK
//  * @value 0 - Not Print
//  * @value 1 - Motor Testing
//  * @value 2 - Servo Testing
//  * @value 3 - Motor and Servo Testing
//  * @min 0
//  * @max 3
// */
// PARAM_DEFINE_INT32(A_PFL_MODE, 0);
