/**
 * @file pid.h
 * @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
 * 
 * @brief   PID CONTROL MODULE
 *          - initialize PID structure
 *          - compute PID
 *          - anti-windup filter
 * 
 * 
 * @version 1.4
 * @date 2019-01-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef PID_H
#define PID_H

#include "../utils.h"
#include "../ball_tracker/ball_physic.h"

//_ Data ____________________________

typedef struct PID_t{
    float Kp, Ki, Kd;
    uint16_t setpoint;
    short error[2];
    float dt;
    uint16_t output[2];
    short integral;
    uint16_t min, max;
    bool inverted_mode;
}PID_t;


//_ Function Signature _________________________

PID_t createPID(float Kp, float Ki, float Kd, uint16_t setpoint, bool mode,
                uint16_t min_angle, uint16_t max_angle);

void PIDCompute(PID_t* pidX, PID_t* pidY, Ball ball);

short saturationFilter(short value , short T_MIN, short T_MAX);

void printPID(PID_t pid);


#endif
