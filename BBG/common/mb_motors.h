/*******************************************************************************
* mb_motors.h
*******************************************************************************/

#ifndef MB_MOTORS_H
#define MB_MOTORS_H

#include <rc_usefulincludes.h> 
#include <roboticscape.h>
#include "mb_defs.h"

//fuctions
int mb_initialize_motors();
int mb_enable_motors();
int mb_disable_motors();
int mb_set_motor(int motor, float duty);
int mb_set_motor_all(float duty);

#endif