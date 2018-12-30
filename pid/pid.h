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
    float setpoint;
    float tollerance;   //in pixels
    float dt;
    float P, I, D;
    float kp, ki, kd;
    float error;
    float pre_error;
}PID_t;

#define   P_MAX   40
#define   P_MIN   -40
#define   I_MIN   -10
#define   I_MAX   10
#define   D_MIN   -5
#define   D_MAX   5

//_ Function Signature _________________________

void setPid(PID_t* pid, float setpoint, float tollerance, float dt, 
            float P, float I, float D, float kp, float kd, float ki, 
            float pre_error, float error);


bool PIDCompute(  PID_t* XPid, PID_t* YPid,
                  Point_t* ball_position,
                  ServoConfig_t* config);



#endif