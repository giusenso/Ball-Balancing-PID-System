#ifndef PID_H
#define PID_H

#include "../utils.h"


//_ Data ____________________________

//not used yet
typedef enum{
  PID_Mode_Automatic = 1,
  PID_Mode_Manual    = 0
} PidModeType;

//not used yet
typedef enum{
  PID_Direction_Direct  = 0,
  PID_Direction_Reverse = 1
} PidDirectionType;


typedef struct PID{
    float setpoint;
    float tollerance;   //in pixels
    float dt;
    float P, I, D;
    float kp, ki, kd;
    float error;
    float pre_error;
}PID_t;

#define   P_MAX   600
#define   P_MIN   -600
#define   I_MIN   -150
#define   I_MAX   150
#define   D_MIN   -50 //not used
#define   D_MAX   50 //not used

//_ Function Signature _________________________

void setPid(PID_t* pid, float setpoint, float tollerance, float dt, 
            float P, float I, float D, float kp, float kd, float ki, 
            float pre_error, float error);


float PIDCompute(PID_t* pid, int ball_position);

void printPID(PID_t* pid);


#endif
