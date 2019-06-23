#ifndef PID_H
#define PID_H

#include "../utils.h"

typedef struct BallAxis_t{
    uint16_t    p[3];
    short       _dp;
} BallAxis_t;

typedef struct Ball_t {
    bool        detected;
    BallAxis_t  xb;
    BallAxis_t  yb;
} Ball_t;

//_ Data ____________________________

typedef struct PID_t{
    float Kp, Ki, Kd;
    uint16_t setpoint;
    short error[2];
    uint16_t dt;    //in millisec
    uint16_t output[2];
    short integral;
    uint16_t min, max;
    bool inverted_mode;
}PID_t;


//_ Function Signature _________________________



#endif
