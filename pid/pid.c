
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

/* create() **********************************************************************/
PID_t createPID(short _Kp, short _Kd, short _Ki, uint16_t setpoint){
    PID_t pid = {
        .Kp         =   _Kp,
        .Ki         =   _Ki,
        .Kd         =   _Kd,
        .setpoint   =   setpoint,
        .error      =   1,
        .pre_error  =   0,
        .dt         =   0.018,
        .output     =   0,
        .integral   =   0,
        .min        =   MIN_ANGLE,
        .max        =   MAX_ANGLE
    };
    return pid;
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

//print routine for debugging
void printPID(PID_t pid){
    printf("*********************************************\n\
            error: %d\n\
            dt:    %.4lf\n\
            integr:%d\n\
            output:%d\n*********************************************\n\n",
            pid.error, pid.dt, pid.integral, pid.output);

}
