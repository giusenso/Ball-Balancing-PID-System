
/********************************************
*	@Author: Giuseppe Sensolini Arra'		*
*											*
*	PID CONTROL MODULE				        *
*	task:									*
*		- define PID structure  			*
*		- compute PID           			*
*		- debug print   					*
*											*
*********************************************/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "pid.h"

//_ Function Declarations _____________________

/* Setup() **********************************************************************/
void setPid(PID_t* pid, float setpoint,
            float Kp, float Kd, float Ki,
            float pre_error, float error, float dt,
            uint16_t min, uint16_t max){

    pid->setpoint   =   setpoint;
    pid->pre_error  =   pre_error;
    pid->error      =   error;
    pid->output     =   0; 
    pid->integral   =   0;
    pid->Kp =   Kp;
    pid->Kd =   Kd;
    pid->Ki =   Ki;
    pid->dt =   dt;
    pid->min=   min;
    pid->max=   max;
}

/* Compute() **********************************************************************
 *   This function should be called every time "void loop()" executes.
 *   the function will decide for itself whether a new
 *   pid Output needs to be computed.
 * 
 *   note: error is expressed in pixels, control should be a duty cicle,
 *   so we need to map this gap with constant:
 *       W(multiply) and Q(offset) 
 **********************************************************************************/
//  (pid, ball) ==> P+I+D

float PIDCompute(PID_t* pid, uint16_t ball_position) {
    
    //set old err
    pid->pre_error = pid->error;

    //compute new error
    pid->error = (pid->setpoint-ball_position);

    //update integral
    pid->integral += pid->error * pid->dt; 

    pid->output =
            HALF_ANGLE +
            pid->Kp * pid->error +
            pid->Ki * pid->integral + 
            pid->Kd * ((pid->error-pid->pre_error)/pid->dt);
    
    if(pid->output<MIN_ANGLE) pid->output=MIN_ANGLE;
    else if(pid->output>MAX_ANGLE) pid->output=MAX_ANGLE;

    return pid->output;
}

//this inline function do all the work
inline void makeServoConfig(ServoConfig_t* config, PID_t* XPID, PID_t* YPID, Point_t ball_pos){
    config->servoX = (uint16_t)PIDCompute(XPID, ball_pos.x);
    config->servoY = (uint16_t)PIDCompute(YPID, ball_pos.y);
}

//print routine for debugging purpouse
void printPID(PID_t* pid){
    printf("\n    err = %lf\n   pre_err = %lf   \n    P+I+D    = %lf",
            pid->error, pid->pre_error,
            pid->output);
}
