/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"

#define DTHETA_THRESH 0.001

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->theta = theta;	
	mb_odometry ->v = 0;
	mb_odometry ->w = 0;
}


/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
	// note omega_filtered is using the median filter
	float delta_left = (WHEEL_DIAMETER * PI * mb_state->left_encoder) / (ENCODER_RES*GEAR_RATIO); 
	//printf("Delta left is : %6.3f \n", delta_left);
	float delta_right = (WHEEL_DIAMETER * PI * mb_state->right_encoder) / (ENCODER_RES*GEAR_RATIO); 
	//printf("Difference is : %6.5f \n", delta_right-delta_left);
	float delta_d =  (delta_left + delta_right)/2.0;

	float delta_theta_odo = (delta_right - delta_left)/WHEEL_BASE;
	//
	mb_odometry -> x += delta_d * cos(mb_odometry -> theta);
	mb_odometry -> y += delta_d * sin(mb_odometry -> theta);
	mb_odometry -> v = delta_d*25;
	mb_odometry -> w = delta_theta_odo*25;
	//printf("x is : %6.5f \n",mb_odometry -> x  );
	// update the theta 
	//if (fabs(mb_state->theta - mb_odometry->theta - delta_theta_odo ) > 50000.0){
	//	mb_odometry -> theta  = mb_state -> theta;
	//}
	//else 
	mb_odometry -> theta = mb_odometry -> theta + delta_theta_odo;


}


/*******************************************************************************
* mb_clamp_radians() 
*
* TODO: clamp and angle between -pi and pi
*
*******************************************************************************/
float mb_clamp_radians(float angle){
    return 0;
}