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
    float setpoint; //px coordinate
    float Kp, Ki, Kd;
    float error;
    float pre_error;
    float dt;
    float output;
    float integral;
    uint16_t min, max;
}PID_t;

#define   P_MAX   600
#define   P_MIN   -600
#define   I_MIN   -150
#define   I_MAX   150
#define   D_MIN   -50 //not used
#define   D_MAX   50 //not used

//_ Function Signature _________________________

void setPid(PID_t* pid, float setpoint,
            float Kp, float Kd, float Ki,
            float pre_error, float error, float dt,
            uint16_t min, uint16_t max);


float PIDCompute(PID_t* pid, int ball_position);

void printPID(PID_t* pid);


#endif
