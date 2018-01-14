// Define low pass filters for left and right motors
rc_filter_t leftlowpassfilt;
rc_filter_t rightlowpassfilt;

// temp variables 
float left_Omega = 0.0;
float right_Omega = 0.0;
int left_encoder_stage2 = 0;
int right_encoder_stage2 = 0;

// lcm related variables
lcm_t * lcm = NULL;
balancebot_msg_t bb_msg;

int left_Encoder;
int right_Encoder;
// moving median
#define MEDIAN_BUFFER_SIZE		5
int left_omega_buffer[MEDIAN_BUFFER_SIZE] = {0};
int right_omega_buffer[MEDIAN_BUFFER_SIZE] = {0};
int left_omega_buffer_to_sort[MEDIAN_BUFFER_SIZE] = {0};
int right_omega_buffer_to_sort[MEDIAN_BUFFER_SIZE] = {0};

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

	//rc_enable_saturation()


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
	imu_config.orientation = ORIENTATION_Z_DOWN;


	// gyro calibration!
	// rc_calibrate_gyro_routine(); // note, need to rerun this !!!!!!!!!!!!!!!!!!!1

	// setup LCM
	// init_bb_msg(&bb_msg);
	pose_xyt_t init_pose = {.utime=0, .x=0, .y=0, .theta=0}; 
    bb_msg.utime = init_pose.utime;
    bb_msg.pose = init_pose;
    bb_msg.num_gates = NUM_OF_GATES;
    bb_msg.gates = (balancebot_gate_t*)malloc(sizeof(balancebot_gate_t)* bb_msg.num_gates);

    // // create lcm object, subscriber thread
    lcm = lcm_create(NULL);
    pthread_t lcm_subscribe_thread;
    pthread_create(&lcm_subscribe_thread, NULL, wrapped_lcm_subscribe_loop,
                    (void*) NULL);

	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}

	rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_initialize_controller();  // load the parameters

	printf("initializing motors...\n");
	mb_initialize_motors();

	printf("resetting encoders...\n");
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	printf("initializing odometry...\n");
	mb_initialize_odometry(&mb_odometry, 0.0,0.0,0.0);

	printf("enabling motors...\n");
	mb_enable_motors();

	// Read PID config
	mb_initialize_controller(); // have left_pid, right_pid, balance_pid
	
	set_log_header();

	// set filters
	leftlowpassfilt = rc_empty_filter();
	rightlowpassfilt = rc_empty_filter();
	

	
	rc_first_order_lowpass(&leftlowpassfilt, DT, 10*DT); // need to tune the tau
	rc_first_order_lowpass(&rightlowpassfilt, DT, 10*DT);
	

	rc_prefill_filter_inputs(&leftlowpassfilt, 0.0);
	rc_prefill_filter_outputs(&leftlowpassfilt, 0.0);
	rc_prefill_filter_inputs(&rightlowpassfilt, 0.0);
	rc_prefill_filter_outputs(&rightlowpassfilt, 0.0);
	// initialize the omega state
	mb_state.left_omega_raw = 0.0;
	mb_state.right_omega_raw = 0.0;
	mb_state.cycle_count = 1;
	mb_state.stop_condition = 0;
	
	// initialize tasks
	// init_task1_desired_position(&mb_state);
	mb_state.task1_point_flag = 1;
    mb_state.task1_desired_position[0] = 0.8;
	mb_state.task1_desired_position[1] = 0.0;
	init_task2_desired_position(&mb_state);

	init_task4_desired_position(&mb_state, &bb_msg);
	// mb_state.task4_waypoints [0][0] = 0.8;
	// mb_state.task4_waypoints [0][1] = 0.0;
	// mb_state.task4_waypoints [1][0] = 0.0;
	// mb_state.task4_waypoints [1][1] = 0.8;	
	
	mb_state.start_position[0]  = 0;
	mb_state.start_position[1]  = 0;

	printf("attaching imu interupt...\n");
	rc_set_imu_interrupt_func(&balancebot_controller);



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
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// creat a state file
	FILE* stateFile;
	// Read IMU and update the state
	mb_state.alpha = imu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.theta = imu_data.dmp_TaitBryan[TB_YAW_Z];

	// Read raw encoders
	int left_Encoder_temp = ENC_1_POL * rc_get_encoder_pos(1);
	//printf("left_Encoder_temp os : %6.3f \n", left_Encoder_temp);
	int right_Encoder_temp = ENC_2_POL * rc_get_encoder_pos(2);
	int i = 0;
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
	//printf("left_Encoder_temp is : %d \n", left_Encoder_temp);

    
    // calculate raw omega
 	mb_state.left_omega_raw = encoderToOmega(left_Encoder);
    mb_state.right_omega_raw = encoderToOmega(right_Encoder);

	// process the encoder data
	if (USE_LOW_PASS_FILTER_ON_ENCODER_VALUE) {
		mb_state.left_encoder = rc_march_filter(&leftlowpassfilt, left_Encoder);
		mb_state.right_encoder = rc_march_filter(&rightlowpassfilt, right_Encoder);
	} else {
		applyMedianFilter(left_Encoder, right_Encoder, &left_Encoder, &right_Encoder);
		mb_state.left_encoder = left_Encoder;
		mb_state.right_encoder = right_Encoder;
	}

    // calculated omega
    mb_state.left_omega_filtered = encoderToOmega(mb_state.left_encoder);
    mb_state.right_omega_filtered = encoderToOmega(mb_state.right_encoder);
    mb_state.left_velocity_filtered = omegaToVelocity(mb_state.left_omega_filtered);
    mb_state.right_velocity_filtered = omegaToVelocity(mb_state.right_omega_filtered);

    
    // update the bot velocity
    mb_state.bot_velocity = 0.5 *(mb_state.left_velocity_filtered + mb_state.right_velocity_filtered);


    mb_update_odometry(&mb_odometry, &mb_state);

    // Calculate controller outputs
    mb_controller_update(&mb_state, &mb_odometry, &mb_setpoints);

    //mb_controller_update_dutytest(&mb_state);
    
    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);

    // Print state
    stateFile = fopen("botstate.csv", "a");
    


 	fprintf(stateFile,"%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f  \n", 
 		mb_state.left_omega_raw, 
 		mb_state.left_omega_filtered, 
 		mb_state.right_omega_raw, 
 		mb_state.right_omega_filtered, 
 		mb_state.left_cmd, 
 		mb_state.right_cmd,
 		mb_state.alpha, 
 		mb_state.theta, 
 		mb_state.bot_velocity,
 		mb_odometry.x,
 		mb_odometry.y,
 		mb_odometry.theta,
 		mb_odometry.angle_x_position,
 		mb_odometry.angle_x_dest,
 		mb_state.norm_distance,
 		mb_state.parallel_distance);
    fflush(stateFile);



    // reset encoders to 0
    rc_set_encoder_pos(1, 0);
    rc_set_encoder_pos(2, 0);
   
    if(!mb_setpoints.manual_ctl){
		
		mb_set_motor(LEFT_MOTOR, GIVE_MOTOR_POWER * MOT_1_POL*mb_state.left_cmd);
		mb_set_motor(RIGHT_MOTOR, GIVE_MOTOR_POWER * MOT_2_POL*mb_state.right_cmd);

   	}

    if(mb_setpoints.manual_ctl){
   		mb_state.left_cmd -= mb_setpoints.turn_velocity;
    	mb_state.right_cmd += mb_setpoints.turn_velocity;
   		mb_set_motor(LEFT_MOTOR, GIVE_MOTOR_POWER * MOT_1_POL*mb_state.left_cmd);
		mb_set_motor(RIGHT_MOTOR, GIVE_MOTOR_POWER * MOT_2_POL*mb_state.right_cmd);
   	}
    // TODO: Set motor velocities

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
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

			mb_setpoints.fwd_velocity = FWD_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(3);
			mb_setpoints.turn_velocity = TURN_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(4);
			if(rc_get_dsm_ch_normalized(5) > 0.0){
				mb_setpoints.manual_ctl = 1;
			}
			else{
				mb_setpoints.manual_ctl = 0;
			}

		}	
			usleep(1000000 / RC_CTL_HZ);
	}
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
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |           ODOMETRY          |");
			printf("\n");
			printf("%11s|","α");
			printf("%11s|","θ_imu");
			printf("%10s|","X");
			printf("%10s|","Y");
			printf("%11s|","θ_odo");
			printf("%10s|","ang_x_pos");
			printf("%10s|","ang_x_dest");
			printf("%10s|","head_err");
			printf("%10s|","norm");
			printf("%10s|","parallel");
			printf("%10s|","point flag");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			printf("%10.3f|", mb_state.alpha*R2D);
			printf("%10.3f|", mb_state.theta*R2D);
			printf("%10.3f|", mb_odometry.x);
			printf("%10.3f|", mb_odometry.y);
			printf("%10.3f|", mb_odometry.theta*R2D);
			printf("%10.3f|", mb_odometry.angle_x_position*R2D);
			printf("%10.3f|", mb_odometry.angle_x_dest*R2D);
			printf("%10.3f|", mb_odometry.heading_error*R2D);
			printf("%10.3f|", mb_state.norm_distance);
			printf("%10.3f|", mb_state.parallel_distance);
			printf("%10d|", mb_state.task1_point_flag);
			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
} 

int comp (const void * elem1, const void * elem2) 
{
    float f = abs(*((float*)elem1));
    float s = abs(*((float*)elem2));
    if (f > s) return  1;
    if (f < s) return -1;
    return 0;
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

void init_bb_msg(balancebot_msg_t * bb_msg) {
    pose_xyt_t init_pose = {.utime=0, .x=0, .y=0, .theta=0}; 
    bb_msg -> utime = init_pose.utime;
    bb_msg -> pose = init_pose;
    bb_msg -> num_gates = NUM_OF_GATES;
    bb_msg -> gates = (balancebot_gate_t*)malloc(sizeof(balancebot_gate_t)* bb_msg -> num_gates);

    
}

// function called by lcm subscription loop on channel OPTITRACK_CHANNEL
void wrapped_beaglebone_message_handler(const lcm_recv_buf_t* rbuf,
                               const char* channel,
                               const balancebot_msg_t* msg,
                               void* userdata){

   // lock the mutex
   pthread_mutex_lock(&state_mutex);
   bb_msg = *(balancebot_msg_t_copy(msg));
   pthread_mutex_unlock(&state_mutex);
}

// define lcm subscription thread
void *wrapped_lcm_subscribe_loop(void *data){
    // pass in lcm object instance, channel from which to read from
    // function to call when data receiver over the channel,
    // and the lcm instance again?
    balancebot_msg_t_subscribe(lcm,
                               OPTITRACK_CHANNEL,
                               wrapped_beaglebone_message_handler,
                               lcm);

    while(1){
        // define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        usleep(1000000 / LCM_HZ);
    }
    lcm_destroy(lcm);
    return 0;
}


// define loop for printing information
void* wrapped_lcm_printf_loop(void* ptr){

    printf("\nPose (X,Y,Z) - # Gates - Next Gate (L-(X,Y), R-(X,Y)) : \n");
    while(rc_get_state()!=EXITING){
        printf("\r");
        int i;
        printf("(%4.2f, %4.2f, %4.2f) - %2d",
                        bb_msg.pose.x, 
                        bb_msg.pose.y, 
                        bb_msg.pose.theta,
                        bb_msg.num_gates);
                        fflush(stdout);
        for (i = 0; i < bb_msg.num_gates; i ++) {
            printf("((%4.2f, %4.2f), (%4.2f, %4.2f))\n",
                        bb_msg.gates[i].left_post[0],
                        bb_msg.gates[i].left_post[1],
                        bb_msg.gates[i].right_post[0],
                        bb_msg.gates[i].right_post[1]);
                fflush(stdout);
            }
            usleep(1000000 / PRINTF_HZ);
    }
    return NULL;
}