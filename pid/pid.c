#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "pid.h"


//_ Function Declarations _____________________

/* Setup() **********************************************************************/
void setPid(PID_t* pid, float setpoint, float tollerance, float sample, 
            float P, float I, float D, float kp, float ki, float kd){

    pid->setpoint[0] = setpoint;
    pid->tollerance = tollerance;
    pid->sample = sample;
    pid->P = 0.0;
    pid->I = 0.0;
    pid->D = 0.0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->iState = 0.0;
    pid->e[0] = 0.0;
    pid->e[1] = 0.0;
    

}

/* Compute() **********************************************************************
 *   This function should be called every time "void loop()" executes.
 *   the function will decide for itself whether a new
 *   pid Output needs to be computed. Returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PIDCompute(PID_t* pid, short ball_position, short* output) {
    
    // set old err e[1]
    pid->e[1] = pid->e[0];

    // calculate new error e[0]
    pid->e[0] = pid->setpoint[0] - ball_position;

    // if the ball is out of X target area: apply control law
    if (fabsf(pid->e[0]) > pid->tollerance ){
     
    //__Proportional term_____________________________________
        pid->P = pid->kp * pid->e[0];
        if (pid->P < P_MIN) pid->P = P_MIN;
        else if (pid->P > P_MAX) pid->P = P_MAX;

    //__Integral term_________________________________________
        //calculate the integrator state
        pid->iState += pid->e[0];

        //integral anti-windup
        if (pid->iState < I_MIN) pid->iState = I_MIN;
        else if (pid->iState > I_MAX) pid->iState = I_MAX;

        //calculate the integral term
        pid->I = pid->ki * pid->iState;

    //__Terms Sum_____________________________________________
        *output = (short)(pid->P + pid->I);
        return true;
    }

    else return false;
}



void printPidState(PID_t* pid){
    printf("\n     ========================\n");
    printf("    ||	sp[0]:  %f    \n", pid->setpoint[0]);
	printf("    ||	P:      %f    \n", pid->P);
	printf("    ||	I:      %f    \n", pid->I);
	printf("    ||	iState: %f    \n", pid->iState);
	printf("    ||	e[0]:   %f    \n", pid->e[0]);
    printf("    ||	e[1]:   %f    \n", pid->e[1]);
	printf("     ========================\n");

}

void updateSetpoint(PID_t* pid, float sp){
    //save old setpoint
    pid->setpoint[1] = pid->setpoint[0];
    //set current setpoin
    pid->setpoint[0] = sp;
}
