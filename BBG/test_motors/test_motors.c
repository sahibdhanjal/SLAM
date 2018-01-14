/*******************************************************************************
* test_motors.c
*
* 
*******************************************************************************/
#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	//set cpu freq to max performance
	rc_set_cpu_freq(FREQ_1000MHZ);

	
	mb_initialize_motors();
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	mb_enable_motors();

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			//run right forward for 1s
			mb_set_motor(RIGHT_MOTOR, 0.5);
			mb_set_motor(LEFT_MOTOR, 0.0);
			rc_nanosleep(1E9);
			//run left forward for 1s
			mb_set_motor(RIGHT_MOTOR, 0.0);
			mb_set_motor(LEFT_MOTOR, 0.5);
			rc_nanosleep(1E9);
			//run left backwards for 1s
			mb_set_motor(RIGHT_MOTOR, 0.0);
			mb_set_motor(LEFT_MOTOR, -0.5);
			rc_nanosleep(1E9);
			//run right backwards for 1s
			mb_set_motor(RIGHT_MOTOR, -0.5);
			mb_set_motor(LEFT_MOTOR, 0.0);
			rc_nanosleep(1E9);

			//stop motors for 1s
			mb_set_motor(LEFT_MOTOR, 0.0);
			mb_set_motor(RIGHT_MOTOR, 0.0);
			rc_nanosleep(1E9);
		}
		else if(rc_get_state()==PAUSED){
			// do other things
		}
		// sleep
		usleep(100000);
	}
	
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	 
	return 0;
}