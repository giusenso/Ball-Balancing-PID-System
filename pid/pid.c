
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
PID_t createPID(short _Kp, short _Ki, short _Kd, uint16_t setpoint, bool mode){
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
        .max        =   MAX_ANGLE,
        .inverted_mode = mode
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
    if (!pid->inverted_mode) pid->error = (pid->setpoint-ball_position);
    else pid->error = (ball_position-pid->setpoint);

    //Integral: update and filter
    pid->integral += pid->error * pid->dt;
    pid->integral = saturationFilter(pid->integral, -100, +100); 

    //Derivative: Update and filter
    short filtered_dp = smoothingFilter(ball_position, 150);
    filtered_dp = saturationFilter(filtered_dp, -100, +100);

    //output
    pid->output =
            HALF_ANGLE +
            pid->Kp * pid->error +
            pid->Ki * pid->integral + 
            pid->Kd * (filtered_dp/pid->dt);
    

    pid->output = saturationFilter(pid->output, MIN_ANGLE, MAX_ANGLE);

    return pid->output;
}
//_________________________________________________________________________
//this filter return dPosition smoothed
//[T is the threshold in terms of max deviation allowed]
inline short smoothingFilter(uint16_t* pos, uint16_t T){
    short dp = pos[0] - pos[1];
    if(dp > T || dp < -T) return (pos[1]-pos[2]);
    else{
        return dp * 0.45 +
        (pos[1]-pos[2]) * 0.25 +
        (pos[2]-pos[3]) * 0.15 +
        (pos[3]-pos[4]) * 0.1  +
        (pos[4]-pos[5]) * 0.05;
    } 
}

// Correct saturated values
inline short saturationFilter(short value , short T_MIN, short T_MAX){
    if (value <= T_MIN) return T_MIN;
    if (value >= T_MAX) return T_MAX;
    else return value;
}
//_________________________________________________________________________
//print routine for debugging
void printPID(PID_t pid){
    printf("*********************************************\n\
            KP = %d , Ki = %d, KD = %d \n\
            D = %lf\n\
            error: %d\n\
            dt:    %.4lf\n\
            integr:%d\n\
            output:%d\n*********************************************\n\n",
            pid.Kp, pid.Ki, pid.Kd, pid.Kd * ((pid.error-pid.pre_error)/pid.dt),
            pid.error, pid.dt, pid.integral, pid.output);

}
