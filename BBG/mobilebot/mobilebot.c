/*******************************************************************************
* mobilebot.c
*
* Main template code for the mobileBot
* 
*******************************************************************************/
#include "mobilebot.h"
float left_Omega = 0.0;
float right_Omega = 0.0;
int left_encoder_stage2 = 0;
int right_encoder_stage2 = 0;

int left_Encoder;
int right_Encoder;
// moving median
#define MEDIAN_BUFFER_SIZE		5
int left_omega_buffer[MEDIAN_BUFFER_SIZE] = {0};
int right_omega_buffer[MEDIAN_BUFFER_SIZE] = {0};
int left_omega_buffer_to_sort[MEDIAN_BUFFER_SIZE] = {0};
int right_omega_buffer_to_sort[MEDIAN_BUFFER_SIZE] = {0};

#define R2D 180/3.1415926

float x_msg;
float y_msg;
float theta_msg;


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

	// start lcm handle thread
	printf("starting lcm thread... \n");
	lcm = lcm_create(LCM_ADDRESS);
	pthread_t lcm_subscribe_thread;
    pthread_create(&lcm_subscribe_thread, NULL, lcm_subscribe_loop, (void*) NULL);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		printf("starting print thread... \n");
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	pthread_create(&setpoint_control_thread, NULL, setpoint_control_loop, (void*) NULL);




	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Z_UP;

	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_initialize_controller();

	printf("initializing motors...\n");
	mb_initialize_motors();

	printf("resetting encoders...\n");
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	printf("initializing odometry...\n");
	mb_initialize_odometry(&mb_odometry, 0.0,0.0,0.0);

	printf("enabling motors...\n");
	mb_enable_motors();

	printf("attaching imu interupt...\n");
	rc_set_imu_interrupt_func(&mobilebot_controller);



	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep

		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	 
	return 0;
}


/*******************************************************************************
* void mobilebot_controller()
*
* discrete-time mobile controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the mobilebot mobiled
* 
*
*******************************************************************************/
void mobilebot_controller(){

    mbot_imu_t imu_msg;
    mbot_encoder_t encoder_msg;
    odometry_t odo_msg;
	
    //lock state mutex
	pthread_mutex_lock(&state_mutex);
	
    // Read IMU
	mb_state.tb_angles[0] = imu_data.dmp_TaitBryan[TB_PITCH_X];
    mb_state.tb_angles[1] = imu_data.dmp_TaitBryan[TB_ROLL_Y];
	mb_state.tb_angles[2] = imu_data.dmp_TaitBryan[TB_YAW_Z];

    imu_msg.tb_angles[0] = imu_data.dmp_TaitBryan[TB_PITCH_X];
    imu_msg.tb_angles[1] = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    imu_msg.tb_angles[2] = imu_data.dmp_TaitBryan[TB_YAW_Z];

    mb_state.temp = imu_data.temp;
    imu_msg.temp = imu_data.temp;
    
    int i;
    for(i=0;i<3;i++){
        mb_state.accel[i] = imu_data.accel[i];
        imu_msg.accel[i] = imu_data.accel[i];
        mb_state.gyro[i] = imu_data.gyro[i];
        imu_msg.gyro[i] = imu_data.gyro[i];
        mb_state.mag[i] = imu_data.mag[i];
        imu_msg.gyro[i] = imu_data.gyro[i];
    }
    // Read raw encoders
	int left_Encoder_temp = ENC_1_POL * rc_get_encoder_pos(1);
	//printf("left_Encoder_temp os : %6.3f \n", left_Encoder_temp);
	int right_Encoder_temp = ENC_2_POL * rc_get_encoder_pos(2);
	i = 0;
	int edge = 48;
	int to_update_encoder = 1;
	while ( left_Encoder_temp > edge 
			|| right_Encoder_temp > edge 
			|| left_Encoder_temp < -edge 
			|| right_Encoder_temp < -edge ){
		left_Encoder_temp = ENC_1_POL * rc_get_encoder_pos(1);
		right_Encoder_temp = ENC_2_POL * rc_get_encoder_pos(2);
		++i;
		if (i == 5) {
			//printf("Sampled %d times!  direct modify\n", i);
			// left_Encoder = max(-edge, min(edge, left_Encoder));
			// right_Encoder = max(-edge, min(edge, right_Encoder));
			to_update_encoder = 0;
			break;
		}
	}
	if(to_update_encoder) {
		left_Encoder = left_Encoder_temp;
		right_Encoder = right_Encoder_temp;
	}

// calculate raw omega
 	// mb_state.left_omega_raw = encoderToOmega(left_Encoder);
  //   mb_state.right_omega_raw = encoderToOmega(right_Encoder);

	applyMedianFilter(left_Encoder, right_Encoder, &left_Encoder, &right_Encoder);
	mb_state.left_encoder = left_Encoder;
	mb_state.right_encoder = right_Encoder;

	// // calculated omega
 //    mb_state.left_omega_filtered = encoderToOmega(mb_state.left_encoder);
 //    mb_state.right_omega_filtered = encoderToOmega(mb_state.right_encoder);
 //    mb_state.left_velocity_filtered = omegaToVelocity(mb_state.left_omega_filtered);
 //    mb_state.right_velocity_filtered = omegaToVelocity(mb_state.right_omega_filtered);




	// Read encoders
	// mb_state.left_encoder = ENC_1_POL * rc_get_encoder_pos(1);
 //    mb_state.right_encoder = ENC_2_POL * rc_get_encoder_pos(2);
    encoder_msg.left_delta = mb_state.left_encoder;
    encoder_msg.right_delta = mb_state.right_encoder;

    mb_state.left_encoder_total += mb_state.left_encoder;
    mb_state.right_encoder_total += mb_state.right_encoder;
    encoder_msg.leftticks = mb_state.left_encoder_total;
    encoder_msg.rightticks = mb_state.right_encoder_total;

    // Update odometry 
    mb_update_odometry(&mb_odometry, &mb_state);

    // Calculate controller outputs
    mb_controller_update(&mb_state);

    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);

    // reset encoders to 0
    rc_set_encoder_pos(1, 0);
    rc_set_encoder_pos(2, 0);
    
    if(!mb_setpoints.manual_ctl){
    	mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
   		mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
   	}

    if(mb_setpoints.manual_ctl){
    	mb_set_motor(RIGHT_MOTOR, mb_setpoints.fwd_velocity + mb_setpoints.turn_velocity);
   		mb_set_motor(LEFT_MOTOR, mb_setpoints.fwd_velocity - mb_setpoints.turn_velocity);
   	}

   	odo_msg.x = mb_odometry.x;
   	odo_msg.y = mb_odometry.y;
   	odo_msg.theta = mb_odometry.theta;

    //publish IMU & Encoder Data to LCM
    mbot_imu_t_publish(lcm, MBOT_IMU_CHANNEL, &imu_msg);
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
    odometry_t_publish(lcm, ODOMETRY_CHANNEL, &odo_msg);
}

/*******************************************************************************
*  optitrack_message_handler()
*
*  handler function for optitrack driver messages
*  optitrack_driver must be running and optitrack must be set up
*
*******************************************************************************/
void optitrack_message_handler(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const pose_xyt_t* pose,
                                void* userdata){

    // lock the state mutex
    pthread_mutex_lock(&state_mutex);
    mb_state.opti_x = pose->x;
    mb_state.opti_y = pose->y;
    mb_state.opti_theta = pose->theta;
    pthread_mutex_unlock(&state_mutex);
}



void update_now(){
	now = rc_nanos_since_epoch()/1000 + time_offset;
}

/*******************************************************************************
*  timesync_handler()
*
*  set time_offset based of difference 
*  between the Pi time and the local time
*
*******************************************************************************/
void timesync_handler(const lcm_recv_buf_t * rbuf, const char *channel,
				const timestamp_t *timestamp, void *_user){

	if(!time_offset_initialized) time_offset_initialized = 1;
	time_offset = timestamp->utime - rc_nanos_since_epoch()/1000;
}



/*******************************************************************************
*  motor_command_handler()
*
* sets motor velocity setpoints from incoming lcm message
*
*******************************************************************************/
void motor_command_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const mbot_motor_command_t *msg, void *user){

	mb_setpoints.fwd_velocity = msg->trans_v;
	mb_setpoints.turn_velocity = msg->angular_v;

}

void odo_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const odometry_t *msg, void *user){

	x_msg = msg->x;
	y_msg = msg->y;
	theta_msg = msg->theta;

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry
*
*  TODO: Use this thread to handle changing setpoints to your controller
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	// start dsm listener for radio control
	rc_initialize_dsm();

	while(1){
		if (rc_is_new_dsm_data()) {
	 		
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may also implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.

		if(rc_get_dsm_ch_normalized(5) > 0.0){
			mb_setpoints.manual_ctl = 1;
			mb_setpoints.fwd_velocity = FWD_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(3);
			mb_setpoints.turn_velocity = TURN_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(4);
		}
		else{
			mb_setpoints.manual_ctl = 0;
		}

	 	}
	 	usleep(1000000 / RC_CTL_HZ);
	}
}


/*******************************************************************************
* lcm_subscribe_loop() 
*
* thread subscribes to lcm channels and sets handler functions
* then handles lcm messages in a non-blocking fashion
*
* TODO: Add other subscriptions as needed
*******************************************************************************/
void *lcm_subscribe_loop(void *data){
    // pass in lcm object instance, channel from which to read from
    // function to call when data receiver over the channel,
    // and the lcm instance again?
    pose_xyt_t_subscribe(lcm,
                         OPTITRACK_CHANNEL,
                         optitrack_message_handler,
                         NULL);

    mbot_motor_command_t_subscribe(lcm, 
    							   MBOT_MOTOR_COMMAND_CHANNEL, 
    							   motor_command_handler, 
    							   NULL);

	timestamp_t_subscribe(lcm, 
						  MBOT_TIMESYNC_CHANNEL, 
						  timesync_handler, 
						  NULL);

	odometry_t_subscribe(lcm,
						ODOMETRY_CHANNEL,
						odo_handler,
						NULL);

    while(1){
        // define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        usleep(1000000 / LCM_HZ);
    }
    lcm_destroy(lcm);
    return 0;
}

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING...\n");
			printf("           SENSORS           |           ODOMETRY          |");
			printf("\n");
			printf("  IMU θ  |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    θ    |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			printf("%7.3f  |", mb_state.tb_angles[2]);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			// printf("%7.3f |", mb_odometry.x);
			// printf("%7.3f  |", mb_odometry.y);
			// printf("%7.3f |", mb_odometry.theta*R2D);
			printf("%7.3f |", x_msg);
			printf("%7.3f  |", y_msg);
			printf("%7.3f |", theta_msg);

			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
} 


void applyMedianFilter(int left_raw_omega, int right_raw_omega, int* left_omega_median, int* right_omega_median) {
	int ii =0;
    for(ii = 0; ii < MEDIAN_BUFFER_SIZE -1; ii++) {
        left_omega_buffer[ii] = left_omega_buffer[ii+1];
        right_omega_buffer[ii] = right_omega_buffer[ii+1];
    }
    left_omega_buffer[MEDIAN_BUFFER_SIZE - 1] = left_raw_omega;
    right_omega_buffer[MEDIAN_BUFFER_SIZE -1] = right_raw_omega;
    for(ii = 0; ii < MEDIAN_BUFFER_SIZE; ii++) {
        left_omega_buffer_to_sort[ii] = left_omega_buffer[ii];
    }
    for(ii = 0; ii < MEDIAN_BUFFER_SIZE; ii++) {
        right_omega_buffer_to_sort[ii] = right_omega_buffer[ii];
    }
    qsort (left_omega_buffer_to_sort,  MEDIAN_BUFFER_SIZE, sizeof(int), comp); 
    qsort (right_omega_buffer_to_sort,  MEDIAN_BUFFER_SIZE, sizeof(int), comp); 
    *left_omega_median = left_omega_buffer_to_sort[MEDIAN_BUFFER_SIZE/2];
    *right_omega_median = right_omega_buffer_to_sort[MEDIAN_BUFFER_SIZE/2];
}

float encoderToOmega(int tick) {
	return ( 2.0 * PI * tick) / (ENCODER_RES * GEAR_RATIO * DT);
}

float omegaToVelocity(float omega) {
	return WHEEL_RADIUS * omega;
}

int comp (const void * elem1, const void * elem2) 
{
    float f = abs(*((float*)elem1));
    float s = abs(*((float*)elem2));
    if (f > s) return  1;
    if (f < s) return -1;
    return 0;
}
