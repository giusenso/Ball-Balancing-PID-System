
/**
 * @file pid.c
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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "pid.h"

/**
 * @brief initialize PID data structure
 * 
 * @param _Kp proportional gain
 * @param _Ki integrative gain
 * @param _Kd derivative gain
 * @param setpoint target
 * @param mode true=>inverted false=>non-inverted
 * @param min_angle minimum servo pulse
 * @param max_angle maximum servo pulse
 * @return PID_t data structure
 */
PID_t createPID(float _Kp, float _Ki, float _Kd,
                uint16_t setpoint, bool mode,
                uint16_t min_angle, uint16_t max_angle){

    PID_t pid = {
        .Kp         =   _Kp,
        .Ki         =   _Ki,
        .Kd         =   _Kd,
        .setpoint   =   setpoint,
        .error      =   { 0,0 },
        .dt         =   0.01,
        .output     =   { 0,0 }, 
        .integral   =   0,
        .min        =   min_angle,
        .max        =   max_angle,
        .inverted_mode = mode
    };
    return pid;
}


/**
 * @brief   (pid, ball) ==> P+I+D
 *   This function should be called every time "loop()" executes.
 *   the function will decide for itself whether a new
 *   pid Output needs to be computed.
 * 
 *   note[1]: error is expressed in pixels, control should be a pulse width
 * 
 *   note[2]: this could be split in two functions that can run parallel,
 *            but thread switch time is probably greater than compute time.
 *            [in any case will be tested]
 * 
 * @param pidX 
 * @param pidY 
 * @param ball 
 */

void PIDCompute(PID_t* pidX, PID_t* pidY, Ball_t ball) {

/*=================================================================================*/
/*      SERVO X     */ 

    //set old error and output
    pidX->error[1] = pidX->error[0];
    pidX->output[1] = pidX->output[0]; 

    //compute new error
    if (!pidX->inverted_mode) pidX->error[0] = pidX->setpoint - ball.x[0];
    else pidX->error[0] = ball.x[0] - pidX->setpoint;

    //Integral: update and filter
    pidX->integral += pidX->error[0] * pidX->dt;
    pidX->integral = saturationFilter(pidX->integral, -150, +150); 

    //Derivative: Update and filter
    ball.smooth_dx = saturationFilter(ball.smooth_dx, -150, +150);
    if(!pidX->inverted_mode) ball.smooth_dx = -ball.smooth_dx;
    
    //output
    pidX->output[0] =
            X_HALF_ANGLE +
            pidX->Kp * pidX->error[0] +
            pidX->Ki * pidX->integral +
            pidX->Kd * (ball.smooth_dx/pidX->dt);

     //output filter   
    pidX->output[0] = saturationFilter(pidX->output[0], pidX->output[1]-1500, pidX->output[1]+1500);
    pidX->output[0] = saturationFilter(pidX->output[0], pidX->min, pidX->max);


/*=================================================================================*/
/*      SERVO Y      */ 

    //set old error and output
    pidY->error[1] = pidY->error[0];
    pidY->output[1] = pidY->output[0]; 

    //compute new error
    if (!pidY->inverted_mode) pidY->error[0] = pidY->setpoint - ball.y[0];
    else pidY->error[0] = ball.y[0] - pidY->setpoint;

    //Integral: update and filter
    pidY->integral += pidY->error[0] * pidY->dt;
    pidY->integral = saturationFilter(pidY->integral, -150, +150); 

    //Derivative: Update and filter
    ball.smooth_dy = saturationFilter(ball.smooth_dy, -150, +150);
    if(!pidY->inverted_mode) ball.smooth_dy = -ball.smooth_dy;
    
    //output
    pidY->output[0] =
            Y_HALF_ANGLE +
            pidY->Kp * pidY->error[0] +
            pidY->Ki * pidY->integral +
            pidY->Kd * (ball.smooth_dy/pidY->dt);
    
    //output filter 
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->output[1]-1500, pidY->output[1]+1500);
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->min, pidY->max);
//===============================================================================
}

/**
 * @brief correct saturated values
 * 
 * @param value 
 * @param T_MIN 
 * @param T_MAX 
 * @return short 
 */
inline short saturationFilter(short value , short T_MIN, short T_MAX){
    if (value <= T_MIN) return T_MIN;
    if (value >= T_MAX) return T_MAX;
    else return value;
}

/**
 * @brief dynamically change setpoint:
 *      this cant be done with a PI-D controller, 
 *      because we need a error derivative instead of position derivative.
 *      To fix this we need to fake a derivative errore here,
 * @param XPID pointer to pid struct
 * @param YPID pointer to pid struct
 * @param sp new setpoint
 * @return true if setpoint is allowed
 * @return false if setpoint is not allowed
 */
bool changeSetpoint(PID_t* XPID, PID_t* YPID, Ball_t* ball, Point_t sp){
    if( ((sp.x > SETPOINT_X+CONTROL_AREA/2) || (sp.x < SETPOINT_X-CONTROL_AREA/2)) &&
        ((sp.x > SETPOINT_X+CONTROL_AREA/2) || (sp.x < SETPOINT_X-CONTROL_AREA/2)) ){
        
        ball->smooth_dx = sp.x - XPID->setpoint;
        XPID->setpoint = sp.x;

        ball->smooth_dy = sp.y - YPID->setpoint;
        YPID->setpoint = sp.y;

        return true;
    }
    else return false;
}

/**
 * @brief debug print
 * 
 * @param pid 
 */
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
