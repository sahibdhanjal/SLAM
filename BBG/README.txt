/*******************************************************************************
*                    Mobilebot Template Code for MBot
*                           pgaskell@umich.edu
*       
*******************************************************************************/

bin/			             : Binaries folder
mobilebot/mobilebot.c/.h     : Main setup and threads
test_motors/test_motors.c/.h : Program to test motor implementation
common/mb_controller.c/.h    : Contoller for manual and autonomous nav
common/mb_defs.h             : Define hardware config
common/mb_motors.c/.h        : Motor functions to be used by balancebot
common/mb_odometry.c/.h	     : Odometry and dead reckoning 
common/mb_pid.c/.h           : Generic PID functions used by mb_controller
lcmtypes/                    : lcmtypes for Mobilebot
java/                        : java build folder for lcmtypes for lcm-spy
optitrack/                   : program for reading Optitrack data and publishing lcm messages
setenv.sh                    : sets up java PATH variables for lcm-spy (run with: source setenv.sh)
