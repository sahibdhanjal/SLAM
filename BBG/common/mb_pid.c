/*******************************************************************************
* mb_pid.c
*
* TODO: implement these functions to build a generic PID controller
*       with a derivative term low pass filter
*
*******************************************************************************/

#include "mb_pid.h"

#define MAX_OUTPUT 1.0
#define MIN_OUTPUT -1.0
#define ITERM_MIN -1.0
#define ITERM_MAX 1.0


PID_t * PID_Init(float Kp, float Ki, float Kd, float dFilterHz, float updateHz) {
  // zero initial values
  PID_t *pid =  malloc(sizeof(PID_t));
  pid->kp = Kp;
  pid->ki = Ki;
  pid->kd = Kd;
  pid->dFilterHz = dFilterHz;
  pid->updateHz = updateHz;

  pid->pidInput = 0;    // input to pid (error)
  pid->pidOutput = 0;   // pid output
  
  pid->pTerm = 0;   // proportional term of output
  pid->iTerm = 0;   // integral term
  pid->dTerm = 0;   // derivative term
  
  pid->prevInput = 0; // previous input for calculating derivative

  pid->iTermMin = ITERM_MIN;
  pid->iTermMax = ITERM_MAX; // integral saturation
  pid->outputMin = MIN_OUTPUT;
  pid->outputMax = MAX_OUTPUT; //pid output saturation
  pid->dFilter = rc_empty_filter(); //low pass filter for DTerm

  return pid;
}

float PID_Compute1(PID_t* pid, float error) { 
     pid->pidInput = rc_march_filter(&(pid->dFilter), error);
    // pid->pidInput = error;
     //if(pid->pidInput*pid->prevInput < 0) PID_ResetIntegrator(pid);

     //pid->pidInput = error;
     pid->pTerm = (pid->kp)*error;
     pid->iTerm += (pid->ki)*error/(pid->updateHz);
     if (pid->iTerm > pid->iTermMax){
      pid->iTerm = pid->iTermMax;
     }
     if (pid->iTerm < pid->iTermMin){
      pid->iTerm = pid->iTermMin;
     }

     pid->dTerm = (pid->pidInput - pid->prevInput)*(pid->kd);
     
     pid->prevInput = pid-> pidInput;
     pid->pidOutput = pid->pTerm + pid->iTerm + pid->dTerm;
     if (pid->pidOutput > pid->outputMax){
      pid->pidOutput = pid->outputMax;
     }
     if (pid->pidOutput < pid->outputMin){
      pid->pidOutput = pid->outputMin;
     }
  return pid->pidOutput;
}

float PID_Compute2(PID_t* pid, float error) { 
     //pid->pidInput = rc_march_filter(&(pid->dFilter), error);
     pid->pidInput = error;
     //if(pid->pidInput*pid->prevInput < 0) PID_ResetIntegrator(pid);

     //pid->pidInput = error;
     pid->pTerm = (pid->kp)*error;
     //if (error*pid->prevInput < 0) PID_ResetIntegrator(pid);
     pid->iTerm += (pid->ki)*error/(pid->updateHz);
     if (pid->iTerm > pid->iTermMax){
      pid->iTerm = pid->iTermMax;
     }
     if (pid->iTerm < pid->iTermMin){
      pid->iTerm = pid->iTermMin;
     }

     pid->dTerm = (error - pid->prevInput)*(pid->kd);
     
     pid->prevInput = pid-> pidInput;
     pid->pidOutput = pid->pTerm + pid->iTerm + pid->dTerm;
     if (pid->pidOutput > pid->outputMax){
      pid->pidOutput = pid->outputMax;
     }
     if (pid->pidOutput < pid->outputMin){
      pid->pidOutput = pid->outputMin;
     }
  return pid->pidOutput;
}


void PID_SetTunings(PID_t* pid, float Kp, float Ki, float Kd) {
  pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
  //scale gains by update rate in seconds for proper units
}

void PID_SetOutputLimits(PID_t* pid, float min, float max){
  pid->outputMin = min;
  pid->outputMax = max;
}

void PID_SetIntegralLimits(PID_t* pid, float min, float max){
  pid-> iTermMin = min;
    pid-> iTermMax = max; 
}

void PID_ResetIntegrator(PID_t* pid){
  pid->iTerm = 0;
}

void PID_SetDerivativeFilter(PID_t* pid, float dFilterHz){
  rc_first_order_lowpass(&(pid->dFilter), 1/pid->updateHz, 1/dFilterHz);
}

void PID_SetUpdateRate(PID_t* pid, float updateHz){
  pid->updateHz = updateHz;
}
