/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "servoCheck.hpp"
#include <px4_platform_common/events.h>
#include <uORB/topics/actuator_servos.h>

using namespace time_literals;

using servo_power_fault_reason_t = events::px4::enums::servo_power_fault_reason_t;
static_assert(servo_power_report_s::SERVO_POWER_FAILURE_COUNT + 1  == (static_cast<uint8_t>(servo_power_fault_reason_t::_max))
	      , "SERVO POWER fault flags mismatch!");
static constexpr const char *servo_power_fault_reason_str(servo_power_fault_reason_t servo_power_fault_reason)
{
	switch (servo_power_fault_reason) {
	case servo_power_fault_reason_t::servo_over_voltage: return "servo_over_voltage";

	case servo_power_fault_reason_t::servo_under_voltage: return "servo_under_voltage";

	case servo_power_fault_reason_t::servo_over_current: return "servo_over_current";

	case servo_power_fault_reason_t::servo_over_current_protect: return "servo_over_current_protect";

	}

	return "";
};

using servo_temp_fault_reason_t = events::px4::enums::servo_temp_fault_reason_t;
static_assert(servo_temp_report_s::SERVO_TEMP_FAILURE_COUNT == (static_cast<uint8_t>(servo_temp_fault_reason_t::_max) + 1)
	      , "SERVO TEMP fault flags mismatch!");
static constexpr const char *servo_temp_fault_reason_str(servo_temp_fault_reason_t servo_temp_fault_reason)
{
	switch (servo_temp_fault_reason) {
	case servo_temp_fault_reason_t::servo_over_heating: return "servo_over_heating";

	case servo_temp_fault_reason_t::servo_under_cooling: return "servo_under_cooling";

	}

	return "";
};


void ServoChecks::checkAndReport(const Context &context, Report &reporter)
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime servo_telemetry_timeout =
		700_ms; // Some DShot SERVOs are unresponsive for ~550ms during their initialization, so we use a timeout higher than that

	servo_status_s servo_status;
	servo_power_status_s servo_power_status;
	servo_temp_status_s servo_temp_status;


	if (_servo_status_sub.copy(&servo_status) && now - servo_status.timestamp < servo_telemetry_timeout) {

		checkServoStatus(context, reporter, servo_status, servo_power_status, servo_temp_status);
		reporter.setIsPresent(health_component_t::actuator_servos);

	} else if (_param_servos_checks_required.get()
		   && now - _start_time > 5_s) { // Wait a bit after startup to allow servo's to init

		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_CHK_SVS</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::actuator_servos, events::ID("check_servos_telem_missing"),
				       events::Log::Critical, "SERVO telemetry missing");
		// reporter.healthFailure(NavModes::All, health_component_t::motors_escs, events::ID("check_escs_telem_missing"),
		// 		       events::Log::Critical, "ESC telemetry missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: SERVO telemetry missing");
		}
	}
}

void ServoChecks::checkServoStatus(const Context &context, Report &reporter, const servo_status_s &servo_status, const servo_power_status_s &servo_power_status, const servo_temp_status_s &servo_temp_status)
{
	const NavModes required_modes = _param_servos_checks_required.get() ? NavModes::All : NavModes::None;

	if (servo_status.servo_count > 0) {

		char servo_fail_msg[50];
		servo_fail_msg[0] = '\0';

		/*	Register online_bitmask with servo_count
			Ex.		servo_status.servo_cservo_statusount = 8
					(1 << servo_status.servo_count)	= 10000000   		//add 0 in 8 digit after 1
					(1 << servo_status.servo_count) - 1 = 10000000 - 1 	//Binary
					online_bitmask = 11111111						// 8 digit of 1
		*/
		int online_bitmask = (1 << servo_status.servo_count) - 1;


		/*
			Ex. 	if servo 2 fail
					online_bitmask = 11111111
					servo_status.servo_online_flags = 11111101
		*/

		// Check if one or more the SERVOs are offline
		if (online_bitmask != servo_status.servo_online_flags) {
			/*	loop check index
				Check value servo_online_flags in index if offline == 0
			*/

			for (int index = 0; index < servo_status.servo_count; index++) {
				if ((servo_status.servo_online_flags & (1 << index)) == 0) {
					uint8_t servo_index = servo_status.servo[index].servo_function - actuator_servos_s::ACTUATOR_FUNCTION_SERVO1 + 1;
					/* EVENT
					 * @description
					 * <profile name="dev">
					 * This check can be configured via <param>COM_ARM_CHK_SVS</param> parameter (servo-itim 138).
					 * </profile>
					 */
					reporter.healthFailure<uint8_t>(required_modes, health_component_t::actuator_servos, events::ID("check_servos_offline"),
									events::Log::Critical, "SERVO {1} offline", servo_index);
					snprintf(servo_fail_msg + strlen(servo_fail_msg), sizeof(servo_fail_msg) - strlen(servo_fail_msg), "SERVO%d ", servo_index);
					servo_fail_msg[sizeof(servo_fail_msg) - 1] = '\0';
				}
			}

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "%soffline. %s\t", servo_fail_msg, context.isArmed() ? "Land now!" : "");
			}
		}


		// Check each servo has fault.
		for (int index = 0; index < servo_status.servo_count; index++) {

			/*servo power status (OVER_VOLTAGE, UNDER_VOLTAGE, OVER_CURRENT, OVER_CURRENT_PROTECT)*/
			if (servo_power_status.servo_power[index].servo_power_error_flags != 0) {

				for (uint8_t power_fault_index = 0; power_fault_index <= static_cast<uint8_t>(servo_power_fault_reason_t::_max); power_fault_index++) {
					if (servo_power_status.servo_power[index].servo_power_error_flags & (1 << power_fault_index)) {

						servo_power_fault_reason_t power_fault_reason_index = static_cast<servo_power_fault_reason_t>(power_fault_index);

						const char *user_action = "";
						events::px4::enums::suggested_action_t action = events::px4::enums::suggested_action_t::none;

						if (context.isArmed()) {
							if ((power_fault_reason_index == servo_power_fault_reason_t::servo_over_current_protect)) {
								user_action = "Land now!";
								action = events::px4::enums::suggested_action_t::land;

							} else {
								user_action = "Land now!";
								action = events::px4::enums::suggested_action_t::land;
							}
						}

						uint8_t servo_index = servo_status.servo[index].servo_function - actuator_servos_s::ACTUATOR_FUNCTION_SERVO1 + 1;

						/* EVENT
						 * @description
						 * {3}
						 *
						 * <profile name="dev">
						 * This check can be configured via <param>COM_ARM_CHK_SVS</param> parameter.
						 * </profile>
						 */
						reporter.healthFailure<uint8_t, events::px4::enums::servo_power_fault_reason_t, events::px4::enums::suggested_action_t>(
							required_modes, health_component_t::actuator_servos, events::ID("check_servos_power_fault"),
							events::Log::Critical, "SERVO {1}: {2}", servo_index, power_fault_reason_index, action);

						if (reporter.mavlink_log_pub()) {
							mavlink_log_emergency(reporter.mavlink_log_pub(), "SERVO%d: %s. %s \t", servo_index,
									      servo_power_fault_reason_str(power_fault_reason_index), user_action);
						}
					}
				}
			}

			/*servo temp status (Over Heat, Over Cooling)*/
			if (servo_temp_status.servo_temp[index].servo_temp_error_flags != 0) {

				for (uint8_t temp_fault_index = 0; temp_fault_index <= static_cast<uint8_t>(servo_temp_fault_reason_t::_max); temp_fault_index++) {
					if (servo_temp_status.servo_temp[index].servo_temp_error_flags & (1 << temp_fault_index)) {

						servo_temp_fault_reason_t temp_fault_reason_index = static_cast<servo_temp_fault_reason_t>(temp_fault_index);

						const char *user_action = "";
						events::px4::enums::suggested_action_t action = events::px4::enums::suggested_action_t::none;

						if (context.isArmed()) {
							if ((temp_fault_reason_index == servo_temp_fault_reason_t::servo_over_heating)) {
								user_action = "Land now!";
								action = events::px4::enums::suggested_action_t::land;

							} else {
								user_action = "Land now!";
								action = events::px4::enums::suggested_action_t::land;
							}
						}

						uint8_t servo_index = servo_status.servo[index].servo_function - actuator_servos_s::ACTUATOR_FUNCTION_SERVO1 + 1;

						/* EVENT
						 * @description
						 * {3}
						 *
						 * <profile name="dev">
						 * This check can be configured via <param>COM_ARM_CHK_SVS</param> parameter.
						 * </profile>
						 */
						reporter.healthFailure<uint8_t, events::px4::enums::servo_temp_fault_reason_t, events::px4::enums::suggested_action_t>(
							required_modes, health_component_t::actuator_servos, events::ID("check_servos_temp_fault"),
							events::Log::Critical, "SERVO {1}: {2}", servo_index, temp_fault_reason_index, action);

						if (reporter.mavlink_log_pub()) {
							mavlink_log_emergency(reporter.mavlink_log_pub(), "SERVO%d: %s. %s \t", servo_index,
									      servo_temp_fault_reason_str(temp_fault_reason_index), user_action);
						}
					}
				}
			}
		}
	}
}

