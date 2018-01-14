#ifndef MB_H
#define MB_H

// usefulincludes is a collection of common system includes
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include <stdlib.h>
#include <string.h>
#include <lcm/lcm.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motors.h"
#include "../common/mb_pid.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/mbot_encoder_t.h"
#include "../lcmtypes/mbot_imu_t.h"
#include "../lcmtypes/mbot_motor_command_t.h"
#include "../lcmtypes/odometry_t.h"
#include "../lcmtypes/oled_message_t.h"
#include "../lcmtypes/timestamp_t.h"



#define     LCM_HZ              100
#define     PRINTF_HZ           10
#define     SETPOINT_HZ         50



#define     OPTITRACK_CHANNEL           "OPTITRACK_CHANNEL"

#define     ODOMETRY_CHANNEL            "ODOMETRY"
#define     CONTROLLER_PATH_CHANNEL     "CONTROLLER_PATH"
#define     MBOT_IMU_CHANNEL            "MBOT_IMU"
#define     MBOT_ENCODER_CHANNEL        "MBOT_ENCODERS"
#define     MBOT_MOTOR_COMMAND_CHANNEL  "MBOT_MOTOR_COMMAND"
#define     MBOT_TIMESYNC_CHANNEL       "MBOT_TIMESYNC"

#define     LCM_ADDRESS                 "udpm://239.255.76.67:7667?ttl=2"


// global variables
lcm_t * lcm;
rc_imu_data_t imu_data;
pthread_mutex_t state_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;
int64_t now;
int64_t time_offset;
int time_offset_initialized;

// functions
void mobilebot_controller();
void update_now();


//LCM handler functions
void optitrack_message_handler(const lcm_recv_buf_t* rbuf,
                               const char* channel,
                               const pose_xyt_t* pose,
                               void* userdata);

void motor_command_handler(const lcm_recv_buf_t *rbuf, 
                                  const char *channel,
                                  const mbot_motor_command_t *msg, 
                                  void *user);

void timesync_handler(const lcm_recv_buf_t * rbuf, 
                             const char *channel,
                             const timestamp_t *timestamp, 
                             void *_user);

//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);
void* lcm_subscribe_loop(void* ptr);
void applyMedianFilter(int left_raw_omega, int right_raw_omega, int* left_omega_median, int* right_omega_median);
float encoderToOmega(int tick);
float omegaToVelocity(float omega);
int comp (const void * elem1, const void * elem2);
#endif