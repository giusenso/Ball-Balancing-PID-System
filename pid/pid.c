#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "pid.h"


//_ Function Declarations _____________________

/* Setup() **********************************************************************/
void setPid(PID_t* pid, float setpoint, float tollerance, float dt,
            float P, float I, float D, float kp, float kd, float ki,
            float pre_error, float error){

    pid->setpoint = setpoint;
    pid->tollerance = tollerance;
    pid->dt = dt;
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->kp = kp;
    pid->kd = kd;
    pid->ki = ki;
    pid->pre_error = pre_error;
    pid->error = error;

}

/* Compute() **********************************************************************
 *   This function should be called every time "void loop()" executes.
 *   the function will decide for itself whether a new
 *   pid Output needs to be computed. Returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
//  (pid, ball) ==> P+I+D
float PIDCompute(PID_t* pid, uint16_t ball_position) {

    //set old err
    pid->pre_error = pid->error;

    //calculate new errors
    pid->error = pid->setpoint - ball_position;

    //if we don't need to compute
    if (fabsf(pid->error) < pid->tollerance){
        pid->P = 0;
        pid->I = 0;
        pid->D = 0;
    }

    //if the ball is out of target area: compute P+i+D
    if (fabsf(pid->error) > pid->tollerance){

        //P term
        pid->P = pid->kp * pid->error;
        if (pid->P < P_MIN) pid->P = P_MIN;
        else if (pid->P > P_MAX) pid->P = P_MAX;

        //I term
        pid->I = pid->ki * pid->error;
        if (pid->I < I_MIN) pid->I = I_MIN;
        else if (pid->I > I_MAX) pid->I = I_MAX;

        /* we don't need D term
        //D term
        pid->D = pid->kd * pid->error;
        if (pid->D < D_MIN) pid->D = D_MIN;
        else if (pid->D > D_MAX) pid->D = D_MAX;
        */
    }

    return pid->P + pid->I + pid->D;
}

//this inline function do all the work
inline void makeServoConfig(ServoConfig_t* config, PID_t* XPID, PID_t* YPID, Point_t ball_pos){
    config->ServoX = (uint8_t)PIDCompute(XPID, ball_pos.x);
    config->ServoY = (uint8_t)PIDCompute(YPID, ball_pos.x);
}


//print routine for debugging purpouse
void printPID(PID_t* pid){
    printf("\n    err = %lf    pre_err = %lf\
            \n    P    = %lf\
            \n    I    = %lf\
            \n    D    = %lf\n",
            pid->error, pid->pre_error,
            pid->P, pid->I, pid->D);
}
