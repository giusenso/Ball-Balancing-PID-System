#ifndef PID_H
#define PID_H

#include "../utils.h"

#define   P_MIN   -600
#define   P_MAX   600
#define   I_MIN   -150
#define   I_MAX   150
#define   D_MIN   -50
#define   D_MAX   50
//_ Data ____________________________

typedef struct PID_t{
    short Kp, Ki, Kd;
    uint16_t setpoint; //px coordinate
    short error;
    short pre_error;
    float dt;
    uint16_t output;
    short integral;
    uint16_t min, max;
}PID_t;


//_ Function Signature _________________________

PID_t createPID(short Kp, short Kd, short Ki, uint16_t setpoint);

float PIDCompute(PID_t* pid, uint16_t ball_position);

void printPID(PID_t pid);


#endif
