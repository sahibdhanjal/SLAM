/*******************************************************************************
* mb_defs.h
*
*   defines for the MBot.
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR              2     // id of left motor
#define RIGHT_MOTOR             1     // id of right motor
#define MDIR1                   60    // gpio1.28  P9.12
#define MDIR2                   48    // gpio1.16  P9.15
#define MOT_EN                  20    // gpio0.20  P9.41
#define MOT_1_POL               1    // polarity of left motor
#define MOT_2_POL               -1   // polarity of right motor
#define ENC_1_POL               1    // polarity of left encoder
#define ENC_2_POL               -1   // polarity of right encoder
#define GEAR_RATIO              34   // gear ratio of motor
#define ENCODER_RES             48   // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER          0.08 // diameter of wheel in meters
#define WHEEL_BASE              0.21  // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     1.0   // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    1.0   // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          25   // main filter and control loop speed
#define DT                      0.04  // 1/sample_rate
#define PRINTF_HZ               10    // rate of print loop
#define RC_CTL_HZ               20    // rate of RC data update
#define WHEEL_RADIUS 			0.04


#endif
