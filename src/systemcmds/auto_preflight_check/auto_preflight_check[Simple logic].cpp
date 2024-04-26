/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file auto_preflight_check.cpp
 *
 * CLI to publish the actuator_test msg
 */
#include "auto_preflight_check.hpp"
bool bypass{false};
AutoPreflightCheck::AutoPreflightCheck() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) //test1 lp_default
{
}

AutoPreflightCheck::~AutoPreflightCheck()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AutoPreflightCheck::init()
{
	// // execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// // alternatively, Run on fixed interval
	// // ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

//Parameters Updater function
void AutoPreflightCheck::Parameters_updated()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
}

//Actuator_test_default_logic
void AutoPreflightCheck::actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}


void AutoPreflightCheck::Run()
{

	float value = 10.0f;
	int m[5] = {0, 3, 1, 2, 4};
	int mt;
	int wave[4] = {5, 7, 9, 11};   // times= (wave[x] - 1)/2    when param input(2,3,4,5)
	int sv_wt = 5; //servo_wave_times
	int mt_times; //how long each motor spinning in once time (default 1.5 sec, max 5.0 sec)
	
	mt_times = _param_auto_pfl_mt_time.get()*1000; //default 1.5s AUTO_PFL_MT_TIME

	if(param_check){
		//Inital Parameter//
		//WIP Stop Inital Parameter Process// i-tim
		if(_param_auto_pfl_mt_time.get()>5.0f||_param_auto_pfl_mt_time.get()<0.5f)
		{
			bypass = false;
			PX4_INFO("auto preflight check Fail");
			PX4_INFO("param AUTO_PFL_MT_TIME is out of range 0.5-5.0 sec");
		}
		if(_param_auto_pfl_mode.get()>3||_param_auto_pfl_mode.get()<0)
		{
			bypass = false;
			PX4_INFO("auto preflight mode Fail");
			PX4_INFO("param AUTO_PFL_MODE is out of range 0-3");
		}

		switch (_param_auto_pfl_sv_loop.get()){
			case 2:
				sv_wt = wave[0];
				break;
			case 3:
				sv_wt = wave[1];
				break;
			case 4:
				sv_wt = wave[2];
				break;
			case 5:
				sv_wt = wave[3];
				break;
			default:
				PX4_INFO("auto preflight check Fail");
				PX4_INFO("param AUTO_PFL_SV_LOOP is out of range 2-5");
				bypass = false;
				break;
			}
		param_check = false;
	}			

	if(bypass){

		PX4_INFO("Started");
		switch (_param_auto_pfl_mode.get()){
		case 0:
			PX4_INFO("Disable Testing Mode");
			break;
		case 1:
			PX4_INFO("Motor Testing");
			for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) 
			{
				value = 0.15f;
				switch (i) {
					case 0:
						mt = m[0];;
						break;
					case 1:
						mt = m[1];
						break;
					case 2:
						mt = m[2];
						break;
					case 3:
						mt = m[3];
						break;
					case 4:
						mt = m[4];
						break;
					default:
						mt =i;
				}

				//PX4_INFO("Real M %i value (%.0f%%)",mt, (double)(value*100.f));
				PX4_INFO("Motor %i Real M %d value (%.0f%%)", i+1, mt, (double)(value*100.f));
				actuator_test(actuator_test_s::FUNCTION_MOTOR1+mt, value, mt_times, false);
				//actuator_test(int function,float value, int timeout_ms, bool release_control)
				px4_usleep(2000000);

				PX4_INFO("Waiting 5 sec to Motor %i",i+2);
				px4_usleep(5000000);
				//a += 1;
			}
	
			PX4_INFO("Motor Testing Done");
			break;

		case 2:
			PX4_INFO("Servo Testing");
			px4_usleep(2000000);
			for (int i = 0; i < 2; ++i) //default i < 2
			{
				value = 0.0f;
				PX4_INFO("Start Servo %i value = (%.0f%%)",i+1,(double)(value*100.f));
				px4_usleep(2000000);
				for (int x = 1; x < sv_wt; ++x) //default x<5
				{
					//float result = std::pow(-1, x + 1);
					float result = 0.0f;
					switch (x % 2) {
						case 0:
							result =-1.0f;
							break;
						case 1:
							result =1.0f;
							break;
						default:
							result =0.0f;
					}
					value = result*0.8f;
					PX4_INFO("Servo %i Test %d value (%.0f%%)", i+1, x, (double)(value*100.f));
					actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 2000, false); //2000
					//actuator_test(int function,float value, int timeout_ms, bool release_control)
					px4_usleep(2000000); //2000000
				}
				if(i == 1){
					bypass = false;
					break;
				}

				PX4_INFO("Waiting 5 sec to Servo %i",i+2);
				px4_usleep(5000000); //Delay to Next Servo 5000000
			}
			PX4_INFO("Servo Testing Done");
			break;

		case 3:
			PX4_INFO("Motor and Servo Testing");
			PX4_INFO("Step [1] Start Motor Test");
			px4_usleep(2000000);
			for (int i = 0; i < 5; ++i) 
			{
				
				value = 0.15f;
				switch (i) {
					case 0:
						mt = m[0];;
						break;
					case 1:
						mt = m[1];
						break;
					case 2:
						mt = m[2];
						break;
					case 3:
						mt = m[3];
						break;
					case 4:
						mt = m[4];
						break;
					default:
						mt =i;
				}

				//PX4_INFO("Real M %i value (%.0f%%)",mt, (double)(value*100.f));
				PX4_INFO("Motor %i Real M %d value (%.0f%%)", i+1, mt, (double)(value*100.f));
				actuator_test(actuator_test_s::FUNCTION_MOTOR1+mt, value, mt_times, false); //mt_times = 1500
				//actuator_test(int function,float value, int timeout_ms, bool release_control)
				px4_usleep(2000000);

				PX4_INFO("Waiting 2 sec to Motor %i",i+2);
				px4_usleep(2000000);
				//a += 1;
			}

			PX4_INFO("Step [2]  Start Servo Test");
			for (int i = 0; i < 2; ++i) 
				{
					value = 0.0f;
					PX4_INFO("Start Servo %i value = (%.0f%%)",i+1,(double)(value*100.f));
					px4_usleep(2000000);
					for (int x = 1; x < 5; ++x) {
							//float result = std::pow(-1, x + 1);
							float result = 0.0f;
							switch (x % 2) {
								case 0:
									result =-1.0f;
									break;
								case 1:
									result =1.0f;
									break;
								default:
									result =0.0f;
							}
							value = result*0.8f;
							PX4_INFO("Servo %i Test %d value (%.0f%%)", i+1, x, (double)(value*100.f));
							actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 2000, false);
							//actuator_test(int function,float value, int timeout_ms, bool release_control)
							px4_usleep(2000000);
						}
					if(i == 1){
						break;
					}

					PX4_INFO("Waiting 2 sec to Servo %i",i+2);
					px4_usleep(2000000);
				}
			PX4_INFO("Finish Motor and Servo test");
			bypass = false;
			return;
			break;
		default:
			break;
		}

		bypass = false;
	}
}

int AutoPreflightCheck::task_spawn(int argc, char *argv[])
{
	AutoPreflightCheck *instance = new AutoPreflightCheck();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AutoPreflightCheck ::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int AutoPreflightCheck ::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AutoPreflightCheck::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test actuators.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Actuator_test", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int auto_preflight_check_main(int argc, char *argv[])
{
	return AutoPreflightCheck::main(argc, argv);
}

/*------------------------------------------------------------*/


// extern "C" __EXPORT int auto_preflight_check_main(int argc, char *argv[]);

// static void actuator_test(int function, float value, int timeout_ms, bool release_control);
// static void usage(const char *reason);


/*void actuator_test*/
// void actuator_test(int function, float value, int timeout_ms, bool release_control)
// {
// 	actuator_test_s actuator_test{};
// 	actuator_test.timestamp = hrt_absolute_time();
// 	actuator_test.function = function;
// 	actuator_test.value = value;
// 	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
// 	actuator_test.timeout_ms = timeout_ms;

// 	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
// 	actuator_test_pub.publish(actuator_test);
// }


// static void usage(const char *reason)
// {
// 	if (reason != nullptr) {
// 		PX4_WARN("%s", reason);
// 	}

// 	PRINT_MODULE_DESCRIPTION(
// 		R"DESCR_STR(
// Utility to test actuators.

// WARNING: remove all props before using this command.
// )DESCR_STR");

// 	PRINT_MODULE_USAGE_NAME("actuator_test", "command");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set an actuator to a specific output value");
// 	PRINT_MODULE_USAGE_PARAM_COMMENT("The actuator can be specified by motor, servo or function directly:");
// 	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8)", true);
// 	PRINT_MODULE_USAGE_PARAM_INT('s', -1, 1, 8, "Servo to test (1...8)", true);
// 	PRINT_MODULE_USAGE_PARAM_INT('f', -1, 1, 8, "Specify function directly", true);

// 	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 0, -1, 1, "value (-1...1)", false);
// 	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout in seconds (run interactive if not set)", true);

// 	PRINT_MODULE_USAGE_COMMAND_DESCR("Motors5-Servos2", "Iterate 4 Motor and 2 Servo (Left-Right Aileron)");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate-motors", "Iterate all motors starting and stopping one after the other");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate-servos", "Iterate all servos deflecting one after the other");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("servos-1-twowave", "Iterate all servos 1-2 (Twowave)");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("Motors-User", "Iterate all Motor In U FR BR BL FL");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("Get-param", "Print param");
	
// }

/*int auto_preflight_check_main*/
// int auto_preflight_check_main(int argc, char *argv[])
// {
// 	int function = 0;
// 	float value = 10.0f;
// 	int ch;
// 	int timeout_ms = 0;
// 	int m[5] = {0, 3, 1, 2, 4};
// 	int mt;
// 	int myoptind = 1;
// 	const char *myoptarg = nullptr;

// 	while ((ch = px4_getopt(argc, argv, "m:s:f:v:t:", &myoptind, &myoptarg)) != EOF) {
// 		switch (ch) {

// 		case 'm':
// 			function = actuator_test_s::FUNCTION_MOTOR1 + (int)strtol(myoptarg, nullptr, 0) - 1;
// 			break;

// 		case 's':
// 			function = actuator_test_s::FUNCTION_SERVO1 + (int)strtol(myoptarg, nullptr, 0) - 1;
// 			break;

// 		case 'f':
// 			function = (int)strtol(myoptarg, nullptr, 0);
// 			break;

// 		case 'v':
// 			value = strtof(myoptarg, nullptr);

// 			if (value < -1.f || value > 1.f) {
// 				usage("value invalid");
// 				return 1;
// 			}
// 			break;

// 		case 't':
// 			timeout_ms = strtof(myoptarg, nullptr) * 1000.f;
// 			break;

// 		default:
// 			usage(nullptr);
// 			return 1;
// 		}
// 	}


// 	if (myoptind >= 0 && myoptind < argc) {
// 		if (strcmp("set", argv[myoptind]) == 0) {

// 			if (value > 9.f) {
// 				usage("Missing argument: value");
// 				return 1;
// 			}

// 			if (function == 0) {
// 				usage("Missing argument: function");
// 				return 1;
// 			}

// 			if (timeout_ms == 0) {
// 				// interactive
// 				actuator_test(function, value, 0, false);

// 				/* stop on any user request */
// 				PX4_INFO("Press Enter to stop");
// 				char c;
// 				ssize_t ret = read(0, &c, 1);

// 				if (ret < 0) {
// 					PX4_ERR("read failed: %i", errno);
// 				}

// 				actuator_test(function, NAN, 0, true);
// 			} else {
// 				actuator_test(function, value, timeout_ms, false);
// 			}
// 			return 0;

// 		} else if (strcmp("iterate-motors", argv[myoptind]) == 0) {
// 			value = 0.15f;
// 			for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) {
// 				PX4_INFO("Motor %i (%.0f%%)", i, (double)(value*100.f));
// 				actuator_test(actuator_test_s::FUNCTION_MOTOR1+i, value, 400, false);
// 				px4_usleep(600000);
// 			}
// 			return 0;

// 		} else if (strcmp("iterate-servos", argv[myoptind]) == 0) {
// 			value = 0.3f;
// 			for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; ++i) {
// 				PX4_INFO("Servo %i (%.0f%%)", i, (double)(value*100.f));
// 				actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 800, false);
// 				px4_usleep(1000000);
// 			}
// 			return 0;
// 		}//itim4
		
		
// 		//
// 		else if (strcmp("servos-1-twowave", argv[myoptind]) == 0) {

// 				for (int i = 0; i < 2; ++i) {
// 					value = 0.0f;
// 					PX4_INFO("Start Servo %i value = (%.0f%%)",i+1,(double)(value*100.f));
// 						px4_usleep(2000000);
// 						for (int x = 1; x < 5; ++x) {
// 								//float result = std::pow(-1, x + 1);
// 								float result = 0.0f;
// 								switch (x % 2) {
// 									case 0:
// 										result =-1.0f;
// 										break;
// 									case 1:
// 										result =1.0f;
// 										break;
// 									default:
// 										result =0.0f;
// 								}
// 								value = result*0.8f;
// 								PX4_INFO("Servo %i Test %d value (%.0f%%)", i+1, x, (double)(value*100.f));
// 								actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 800, false);
// 								//actuator_test(int function,float value, int timeout_ms, bool release_control)
// 								px4_usleep(2000000);
// 						}
// 					if(i == 1){
// 						break;
// 					}

// 					PX4_INFO("Waiting 5 sec to Servo %i",i+2);
// 					px4_usleep(5000000);
// 				}
// 				PX4_INFO("FINISH!!!");
// 			return 0;
// 		}
// 		//itim4

// 		else if (strcmp("Motors5-Servos2", argv[myoptind]) == 0) {
// 			PX4_INFO("Start Motor Test");
// 				px4_usleep(2000000);
// 				for (int i = 0; i < 4; ++i) {
					
// 						value = 0.15f;
// 						switch (i) {
// 									case 0:
// 										mt = m[0];;
// 										break;
// 									case 1:
// 										mt = m[1];
// 										break;
// 									case 2:
// 										mt = m[2];
// 										break;
// 									case 3:
// 										mt = m[3];
// 										break;
// 									case 4:
// 										mt = m[4];
// 										break;
// 									default:
// 										mt =i;
// 								}

// 						//PX4_INFO("Real M %i value (%.0f%%)",mt, (double)(value*100.f));
// 						PX4_INFO("Motor %i Real M %d value (%.0f%%)", i+1, mt, (double)(value*100.f));
// 						actuator_test(actuator_test_s::FUNCTION_MOTOR1+mt, value, 1500, false);
// 						//actuator_test(int function,float value, int timeout_ms, bool release_control)
// 						px4_usleep(2000000);
						

// 					PX4_INFO("Waiting 2 sec to Motor %i",i+2);
// 					px4_usleep(2000000);
// 					//a += 1;
// 				}
// 				PX4_INFO("FINISH!!!");

// 			PX4_INFO("Start Servo Test");
// 				for (int i = 0; i < 2; ++i) {
// 					value = 0.0f;
// 					PX4_INFO("Start Servo %i value = (%.0f%%)",i+1,(double)(value*100.f));
// 						px4_usleep(2000000);
// 						for (int x = 1; x < 5; ++x) {
// 								//float result = std::pow(-1, x + 1);
// 								float result = 0.0f;
// 								switch (x % 2) {
// 									case 0:
// 										result =-1.0f;
// 										break;
// 									case 1:
// 										result =1.0f;
// 										break;
// 									default:
// 										result =0.0f;
// 								}
// 								value = result*0.8f;
// 								PX4_INFO("Servo %i Test %d value (%.0f%%)", i+1, x, (double)(value*100.f));
// 								actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 800, false);
// 								//actuator_test(int function,float value, int timeout_ms, bool release_control)
// 								px4_usleep(2000000);
// 						}
// 					if(i == 1){
// 						break;
// 					}

// 					PX4_INFO("Waiting 2 sec to Servo %i",i+2);
// 					px4_usleep(2000000);
// 				}
// 				PX4_INFO("FINISH!!!");
// 			return 0;
// 		}
// 		//itim4


// 		else if (strcmp("Motors-User", argv[myoptind]) == 0) {

// 				PX4_INFO("Start Motor Test");
// 				px4_usleep(2000000);
// 				for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) {
					
// 						value = 0.15f;
// 						switch (i) {
// 									case 0:
// 										mt = m[0];;
// 										break;
// 									case 1:
// 										mt = m[1];
// 										break;
// 									case 2:
// 										mt = m[2];
// 										break;
// 									case 3:
// 										mt = m[3];
// 										break;
// 									case 4:
// 										mt = m[4];
// 										break;
// 									default:
// 										mt =i;
// 								}

// 						//PX4_INFO("Real M %i value (%.0f%%)",mt, (double)(value*100.f));
// 						PX4_INFO("Motor %i Real M %d value (%.0f%%)", i+1, mt, (double)(value*100.f));
// 						actuator_test(actuator_test_s::FUNCTION_MOTOR1+mt, value, 1500, false);
// 						//actuator_test(int function,float value, int timeout_ms, bool release_control)
// 						px4_usleep(2000000);
						

// 					PX4_INFO("Waiting 5 sec to Motor %i",i+2);
// 					px4_usleep(5000000);
// 					//a += 1;
// 				}
// 				PX4_INFO("FINISH!!!");
// 			return 0;
// 		}
// 		//itim4
// 		else if (strcmp("Get-param", argv[myoptind]) == 0) {

// 				PX4_INFO("Parameter");
// 				//PX4_INFO("Real M %i value (%.0f%%)",mt, (double)(value*100.f));
// 				PX4_INFO("auto_pfl_en value (%.2f)", (double)_param_auto_pfl_en.get());
// 				PX4_INFO("pfl_sv_loop value (%.2f)", (double)_param_pfl_sv_loop.get());
// 				PX4_INFO("pfl_mt_time value (%.2f)", (double)_param_pfl_mt_time.get());

// 			return 0;
// 		}
// 		//itim4

// 	}

// 	usage(nullptr);
// 	return 0;
// }
