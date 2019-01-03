#ifndef PID_H
#define PID_H

#include "../utils.h"


//_ Data ____________________________

typedef enum{
  PID_Mode_Automatic = 1,
  PID_Mode_Manual    = 0
} PidModeType;

typedef enum{
  PID_Direction_Direct  = 0,
  PID_Direction_Reverse = 1
} PidDirectionType;


typedef struct PID{
    float setpoint[2];
    float tollerance;   //in pixels
    float sample;
    float P, I, D;
    float kp, ki, kd;
    float iState;
    float e[2]; //e[0] actual error, e[1] past error
}PID_t;

#define   P_MAX   400
#define   P_MIN   -400
#define   I_MIN   -1000
#define   I_MAX   1000
#define   D_MIN   -500
#define   D_MAX   500

//_ Function Signature _________________________

void printPidState(PID_t* pid);

void setPid(PID_t* pid, float setpoint, float tollerance, float sample, 
            float P, float I, float D, float kp, float ki, float kd);


bool PIDCompute(PID_t* pid, short ball_position, short* ServoX);


void updateSetpoint(PID_t* pid, float sp);

#endif