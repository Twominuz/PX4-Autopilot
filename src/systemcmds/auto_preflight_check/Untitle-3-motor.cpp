if(MOTOR_test){
	if(DO_NEXT_MOTOR){
		DO_MOTOR_SEQUENCE = true;
		IN_MOTOR_SEQUENCE = true;
		DO_NEXT_MOTOR = false;

		PX4_INFO("DO Test MOTOR [%i]",MOTOR_ID+1);
	}

	if((DO_MOTOR_SEQUENCE)&&(IN_MOTOR_SEQUENCE)){
		DO_MOTOR_SEQUENCE = false;
		switch (MOTOR_ID) {
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
				mt =tmn_int1;
		}
		actuator_test(actuator_test_s::FUNCTION_MOTOR1+mt, value, mt_times, false);

		PX4_INFO("Test MOTOR [%i] at SEQUENCE[%i]",MOTOR_ID+1,SEQUENCE);
		++MOTOR_SEQUENCE;

		if(MOTOR_SEQUENCE>=MOTOR_SEQUENCE_MAX){
			IN_MOTOR_SEQUENCE = false;
			MOTOR_SEQUENC = 1;
			
			if((MOTOR_ID+2)<MOTOR_MAX){
				PX4_INFO("Wait Next MOTOR 5 sec.");
			}
			DO_NEXT_MOTOR_DELAY = hrt_absolute_time();
			DO_NEXT_MOTOR_DELAY_TIMER = true;
		}
		else{
			PX4_INFO("Wait MOTOR sequence 2 sec.");
			MOTOR_SEQUENCE_DELAY = hrt_absolute_time();
			MOTOR_SEQUENCE_DELAY_TIMER = true;
		}
	}

	if((MOTOR_SEQUENCE_DELAY_TIMER)&&(hrt_elapsed_time(& MOTOR_SEQUENCE_DELAY) > 2_s)&&(IN_SEQUENCE)){
		MOTOR_SEQUENCE_DELAY_TIMER	= false;
		DO_MOTOR_SEQUENCE = true;
	}

	if((DO_NEXT_MOTOR_DELAY_TIMER)&&(hrt_elapsed_time(& DO_NEXT_MOTOR_DELAY) > 5_s)){
		DO_NEXT_MOTOR_DELAY_TIMER = false;
		DO_NEXT_MOTOR = true;
		++MOTOR_ID;
		if(MOTOR_ID<MOTOR_MAX){
			PX4_INFO("Test Next MOTOR [%i] ",MOTOR_ID+1);
		}
			
	}

	if(MOTOR_ID>=MOTOR_MAX){
		motor_test = false;
		PX4_INFO("****************************************");
		PX4_INFO("Finish");

		//Reset
		DO_NEXT_MOTOR = true;
		DO_MOTOR_SEQUENCE = false;
		IN_MOTOR_SEQUENCE = false;
		DO_NEXT_MOTOR_DELAY_TIMER = false;
		MOTOR_SEQUENCE_DELAY_TIMER = false;
	}
}



if(motor_test){
		//start stamped time
		if ((_time_reset)&&(_time_reset_2)){ //true and start if
			_previous_time = hrt_absolute_time();

			_time_reset = false;
			_time_reset_2 = false;

			//_do_motor_test = false;
			//_time_reset_delay_2 = false;					//reset when do timer laps 1
			// _time_reset = false;
			// _time_reset_delay = false;
			PX4_INFO("---------------------------------");
			//PX4_INFO("Reset [Done]");
			//PX4_INFO("_previous_time %2f, _time_reset_delay_1 = %d", (double)_previous_time, (bool)_time_reset_delay);
			//PX4_INFO("_previous_time %2f, _time_reset_delay_2 = %d", (double)_previous_time, (bool)_time_reset_delay_2);
			PX4_INFO("waiting 3s");
		}

        if ((hrt_elapsed_time(& _previous_time)> 3_s)&&(_time_reset_delay)) //wait 3 sec to start WAITING TIME
		{  //set every n second
            _time_reset = true;
			_do_motor_test = true;

			//reset logic
			_time_reset_delay = false;
			//PX4_INFO("Delay 3s [Done]");
		}
		
		if(_do_motor_test&&(tmn_int1<Num_Motor)){
			//send pwm to motor i
			PX4_INFO("Do_motor");
			value = 0.15f;
				switch (tmn_int1) {
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
						mt =tmn_int1;
				}
			actuator_test(actuator_test_s::FUNCTION_MOTOR1+mt, value, mt_times, false);
			_previous_time_2 = hrt_absolute_time();

			PX4_INFO("Motor %i", tmn_int1+1);
			++tmn_int1;
			//tmn_int1 += 1;
			if(tmn_int1<(Num_Motor)){
				PX4_INFO("Motor Next %i", tmn_int1+1);	
			}

			_time_reset_delay_2 = true;


			//reset logic auto_preflight_check start
			_do_motor_test = false;
			PX4_INFO("RUN TIME = %.1f sec", (double)mt_times/1000);
		}

		if ((hrt_elapsed_time(& _previous_time_2)> (double)(mt_times*1000))&&(_time_reset_delay_2)&&(_time_reset))
		{  //set every n second
			_time_reset_delay = true;
			_time_reset_2 = true;

			//reset logic
			_time_reset_delay_2 = false;
			//PX4_INFO("Timer 5s [Done]");
		}

		if(tmn_int1>=Num_Motor){
			motor_test = false;
			servo_test = true;
			_time_reset = true;
			_time_reset_2 = true;
			_time_reset_delay = true;
			PX4_INFO("****************************************");
			PX4_INFO("Finish");
			//PX4_INFO("Motor now = %d", tmn_int1);
			PX4_INFO("****************************************");
		}
	}





	bool bypass{false};
	bool motor_test{false}; //FUCTION MOTOR
	bool servo_test{true};
	bool param_check{true};
	int tmn_int1 = 0;  // CURRENT MOTOR INDEX
	int tmn_int2 = 0;  // CURRENT servo INDEX
	int tmn_int3 = 1;  // CURRENT servo222 INDEX
	int Num_Motor = 5; //
	int Num_Servo = 2;
	bool _time_reset{true};  //WAITING TIME 
	bool _time_reset_2{true}; //MOTOR RUN TIME
	bool _time_reset_3{true};
	bool _time_reset_delay{true}; 
	bool _time_reset_delay_2{true};
	bool _time_reset_delay_3{false};

	bool _do_motor_test = true;
	bool _do_servo_test = true;
	bool _do_servo_test_wave = true;
    hrt_abstime _previous_time{0};  // COUNTER TIMER  (WAITING TIME)
	hrt_abstime _previous_time_2{0}; //COUNTER TIMER 2  (MOTOR RUN TIME)
	hrt_abstime _previous_time_3{0};
