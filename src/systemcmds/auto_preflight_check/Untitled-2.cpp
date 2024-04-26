
//DO_NEXT_SERVO [EN]
//DO_NEXT_SERVO_DELAY_TIMER [DIS]
//SERVO_ID [0]



/*
bool bypass{false};
bool motor_test{false}; //FUCTION MOTOR
bool servo_test{true};


//SERVO LOGIC
bool DO_NEXT_SERVO{true};
bool DO_SERVO_SEQUENCE{false};
bool IN_SEQUENCE{false};
bool DO_NEXT_SERVO_DELAY_TIMER{false};
bool SERVO_SEQUENCE_DELAY_TIMER{false};

hrt_abstime SERVO_SEQUENCE_DELAY{0};  //(MOTOR RUN TIME)
hrt_abstime DO_NEXT_SERVO_DELAY{0}; //(WAITING TIME)

int Servo_ID = 0;
int SEQUENCE = 0;
int SEQUENCE_MAX = 0;

 */


//SERVO
/* 

        //[DO_NEXT_SERVO]
            /*

                EN      DO_SERVO_SEQUENCE
                EN      IN_SEQUENCE
                DIS     DO_NEXT_SERVO                     
          
            */

		//[DO_SERVO_SEQUENCE]&&[IN_SEQUENCE]
        /*
            DIS DO_SERVO_SEQUENCE
			Start Count _previous_time_2 <SERVO_SEQUENCE_DELAY>	
            
                **SEND Command to SERVO_ID (Value from "SEQUENCE FUCTION")
            
            SEQUENCE++
                if [SEQUENCE == MAX SEQUENCE]
                    DIS IN_SEQUENCE
                    EN  DO_NEXT_SERVO_DELAY_TIMER
                    Start Count _previous_time_3 <DO_NEXT_SERVO_DELAY>
                eles
                    EN  SERVO_SEQUENCE_DELAY_TIMER

		*/



		//[SERVO_SEQUENCE_DELAY_TIMER]&&[SERVO_SEQUENCE_DELAY > step]&&[IN_SEQUENCE]   delay x sec equal to step  ex. step = 2_s 
		/*
			DIS SERVO_SEQUENCE_DELAY_TIMER
            EN  DO_SERVO_SEQUENCE
		
		*/

        //[DO_NEXT_SERVO_DELAY_TIMER]&&[DO_NEXT_SERVO_DELAY > Delay]   delay x sec equal to step  ex. step = 2_s 
		/*
			DIS DO_NEXT_SERVO_DELAY_TIMER
            EN  DO_NEXT_SERVO
            SERVO_ID++
		
		*/

        //[SERVO_ID>=SERVO_MAX]
        /*
			DIS servo_test

            //Rest to Defualt
            DO_NEXT_SERVO{true};
            DO_SERVO_SEQUENCE{false};
            IN_SEQUENCE{false};
            DO_NEXT_SERVO_DELAY_TIMER{false};
            SERVO_SEQUENCE_DELAY_TIMER{false};

		
		*/



/*



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



        
















