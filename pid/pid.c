
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
PID_t createPID(float _Kp, float _Ki, float _Kd,
                uint16_t setpoint, bool mode,
                uint16_t min_angle, uint16_t max_angle){

    PID_t pid = {
        .Kp         =   _Kp,
        .Ki         =   _Ki,
        .Kd         =   _Kd,
        .setpoint   =   setpoint,
        .error      =   {0, 0},
        .dt         =   0.018,
        .output     =   {0, 0}, 
        .integral   =   0,
        .min        =   min_angle,
        .max        =   max_angle,
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

void PIDCompute(PID_t* pidX, PID_t* pidY, Ball ball) {
    
    //set old error and output
    pidX->error[1] = pidX->error[0];
    pidX->output[1] = pidX->output[0]; 

    //compute new error
    if (!pidX->inverted_mode) pidX->error[0] = (pidX->setpoint - ball.x[0]);
    else pidX->error[0] = (ball.x[0] - pidX->setpoint);

    //Integral: update and filter
    pidX->integral += pidX->error[0] * pidX->dt;
    pidX->integral = saturationFilter(pidX->integral, -100, +100); 

    //Derivative: Update and filter
     ball.smooth_dx = saturationFilter(ball.smooth_dx, -50, +50);
    if(!pidX->inverted_mode) ball.smooth_dx = -ball.smooth_dx;
    //printf("filtered_dp = %d\n\n", filtered_dp);
    
    //output
    pidX->output[0] =
            X_HALF_ANGLE +
            pidX->Kp * pidX->error[0] +
            pidX->Ki * pidX->integral +
            pidX->Kd * (ball.smooth_dx/pidX->dt);
    
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->output[1]-600, pidY->output[1]+600);
    pidY->output[0] = saturationFilter(pidY->output[0], pidX->min, pidX->max);
    
    //////////////////////////////////////////////////////////////////////////

    //set old error and output
    pidY->error[1] = pidY->error[0];
    pidY->output[1] = pidY->output[0]; 

    //compute new error
    if (!pidY->inverted_mode) pidY->error[0] = (pidY->setpoint - ball.y[0]);
    else pidY->error[0] = (ball.y[0] - pidY->setpoint);

    //Integral: update and filter
    pidY->integral += pidY->error[0] * pidY->dt;
    pidY->integral = saturationFilter(pidY->integral, -100, +100); 

    //Derivative: Update and filter
    ball.smooth_dy = saturationFilter(ball.smooth_dy, -50, +50);
    if(!pidY->inverted_mode) ball.smooth_dy = -ball.smooth_dy;
    //printf("filtered_dp = %d\n\n", filtered_dp);
    
    //output
    pidY->output[0] =
            Y_HALF_ANGLE +
            pidY->Kp * pidY->error[0] +
            pidY->Ki * pidY->integral +
            pidY->Kd * (ball.smooth_dy/pidY->dt);
    
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->output[1]-600, pidY->output[1]+600);
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->min, pidY->max);
}


//_________________________________________________________________________

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
            KP = %.2lf , Ki = %.2lf, KD = %.2lf \n\
            error: %d\n\
            dt:    %.4lf\n\
            integr:%d\n\
            output:%d\n*********************************************\n\n",
            pid.Kp, pid.Ki, pid.Kd,
            pid.error[0], pid.dt, pid.integral, pid.output[0]);

}
