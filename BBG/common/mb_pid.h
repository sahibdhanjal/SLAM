/*******************************************************************************
* mb_pid.h
*******************************************************************************/

#ifndef MB_PID_H
#define MB_PID_H

#include <rc_usefulincludes.h> 
#include <roboticscape.h>


typedef struct pid_parameters pid_parameters_t;
struct pid_parameters {
    float kp;
    float ki;
    float kd;
    float dFilterHz;
    float out_lim;
    float int_lim;
};

typedef struct pid PID_t;
struct pid{

  float kp; // Proportional Gain
  float ki; // Integral Gain
  float kd; // Derivative Gain

  float pidInput;    // input to pid (error)
  float pidOutput;   // pid output
  
  float pTerm;   // proportional term of output
  float iTerm;   // integral term
  float dTerm;   // derivative term
  
  float prevInput; // previous input for calculating derivative

  float iTermMin, iTermMax; // integral saturation
  float outputMin, outputMax; //pid output saturation

  rc_filter_t dFilter; //low pass filter for DTerm
  float dFilterHz; // lowpass frequency of DTerm filter
  float updateHz; // rate in Hz of pid loop update 
};


//Initialize the PID controller with gains
PID_t* PID_Init(
  float kp,
  float ki,
  float kd,
  float dFilterHz,
  float updateHz
  );
     
float PID_Compute(PID_t* pid, float error);

void PID_SetTunings(PID_t* pid, 
  float kp, 
  float ki, 
  float kd
  );

void PID_SetOutputLimits(PID_t* pid, 
  float min, 
  float max
  );

void PID_SetIntegralLimits(PID_t* pid, 
  float min, 
  float max
  );

void PID_ResetIntegrator(PID_t* pid);

void PID_SetDerivativeFilter(PID_t* pid, 
  float dFilterHz
  );

void PID_SetUpdateRate(PID_t* pid, 
  float updateHz
  );

#endif