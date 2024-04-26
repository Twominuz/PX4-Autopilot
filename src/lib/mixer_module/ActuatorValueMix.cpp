#include "ActuatorValueMix.hpp"

using namespace time_literals;

ActuatorValueMix::ActuatorValueMix(const OutputFunction function_assignments[MAX_ACTUATORS])
	: _function_assignments(function_assignments)
{
	reset();
}

//Parameters Updater function
// void ActuatorValueMix::Parameters_updated()
// {
// 	// Check if parameters have changed
// 	if (_parameter_update_sub.updated()) {
// 		// clear update
// 		parameter_update_s param_update;
// 		_parameter_update_sub.copy(&param_update);
// 		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
// 	}
// }

void ActuatorValueMix::update(int num_outputs, int pfl_mode)
{
	for (int i = 0; i < num_outputs; ++i) {
			new_value = pfl_mode + i;	
	}
}

void ActuatorValueMix::overrideValues(float value[MAX_ACTUATORS], int num_outputs)
{
		for (int i = 0; i < num_outputs; ++i) {

			value[i] = (pfl_mode+0.1f)*0.46f + new_value;

			//value[i] = _current_outputs[i];
			// if(i==num_outputs){
			// 	reset()
			// }
		}
}

void ActuatorValueMix::reset()
{
	// _in_test_mode = false;
	// _next_timeout = 0;

	// for (int i = 0; i < MAX_ACTUATORS; ++i) {
	// 	new_value = NAN;

	// }
}
