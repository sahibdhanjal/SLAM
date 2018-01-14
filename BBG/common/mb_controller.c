#include "mb_controller.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
#define SAMPLE_RATE_HZ 100


int mb_initialize_controller(){

    mb_load_controller_config();
    
    left_pid = PID_Init(
        left_pid_params.kp,
        left_pid_params.ki,
        left_pid_params.kd,
        left_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );

    right_pid = PID_Init(
        right_pid_params.kp,
        right_pid_params.ki,
        right_pid_params.kd,
        right_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
   
    PID_SetOutputLimits(left_pid,-1.0, 1.0);
    PID_SetOutputLimits(right_pid,-1.0, 1.0);

    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    fscanf(file, "%f %f %f %f", 
        &left_pid_params.kp,
        &left_pid_params.ki,
        &left_pid_params.kd,
        &left_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your cascaded PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    return 0;
}