/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "NewWorkItem.hpp"

NewWorkItem::NewWorkItem() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	parameters_update();
}

NewWorkItem::~NewWorkItem()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool NewWorkItem::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}
void NewWorkItem::parameters_update()
{
	if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			updateParams(); // update module parameters (in DEFINE_PARAMETERS)

		}
}
void
NewWorkItem::sorn_parm()
{
	float test_param1 = _param_sn_nwi_val1.get();
	//float test_param2 = _param_sn_nwi_val2.get();
	bool test_EN = _param_sn_nwi_en.get();
	PX4_WARN("Param SN_NWI_VAL1: %.2f",(double)test_param1);
	//PX4_WARN("Param SN_NWI_2: %.2f",(double)test_param2);
	PX4_INFO("Param SN_NWI_1: %d", test_EN);

}

void NewWorkItem::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	parameters_update();

	// Check if parameters have changed

	int value_counter = 0;
	while (value_counter < 100){

		sorn_parm();

	value_counter++;
	px4_usleep(500000);
	}


	// Example
	//  publish some data
	/*
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data); */


	perf_end(_loop_perf);
}

int NewWorkItem::task_spawn(int argc, char *argv[])
{
	NewWorkItem *instance = new NewWorkItem();

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

int NewWorkItem::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NewWorkItem::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NewWorkItem::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("new_work_item", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int new_work_item_main(int argc, char *argv[])
{
	return NewWorkItem::main(argc, argv);
}
