#ifndef PID_H
#define PID_H

#include "../utils.h"

#define   P_MIN   0
#define   P_MAX   40
#define   I_MIN   0
#define   I_MAX   25
#define   D_MIN   0
#define   D_MAX   10
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
    bool inverted_mode;
}PID_t;


//_ Function Signature _________________________

PID_t createPID(short Kp, short Ki, short Kd, uint16_t setpoint, bool mode);

float PIDCompute(PID_t* pid, uint16_t ball_position);

void printPID(PID_t pid);


#endif
