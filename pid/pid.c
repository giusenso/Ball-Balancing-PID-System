#include <math.h>
#include <stdlib.h>

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
bool PIDCompute(PID_t* XPid, PID_t* YPid, Point_t* ball_position, ServoConfig_t* config) {
    
    //set old err
    XPid->pre_error = XPid->error;
    YPid->pre_error = YPid->error;

    //calculate new errors
    XPid->error = XPid->setpoint - ball_position->x;
    YPid->error = YPid->setpoint - ball_position->y;

    if (fabsf(XPid->error) < XPid->tollerance || fabsf(YPid->error) < YPid->tollerance){
        return false;
    }

    // if the ball is out of X target area: apply control law
    if (fabsf(XPid->error) > XPid->tollerance){
        //P term
        XPid->P = XPid->kp * XPid->error;
        if (XPid->P < P_MIN) XPid->P = P_MIN;
        else if (XPid->P > P_MAX) XPid->P = P_MAX;
        //I term
        XPid->I = XPid->ki * XPid->error;
        if (XPid->I < I_MIN) XPid->I = I_MIN;
        else if (XPid->I > I_MAX) XPid->I = I_MAX;
        //D term
        XPid->D = XPid->kd * XPid->error;
        if (XPid->D < D_MIN) XPid->D = D_MIN;
        else if (XPid->D > D_MAX) XPid->D = D_MAX;



        config->ServoX = (uint8_t)(XPid->P + XPid->I + XPid->D);
        
    }

    // if the ball is out of Y target area: apply control law
    if(fabsf(YPid->error) > YPid->tollerance){
        //P term
        YPid->P = YPid->kp * YPid->error;
        if (YPid->P < P_MIN) YPid->P = P_MIN;
        else if (YPid->P > P_MAX) YPid->P = P_MAX;
        //I term
        YPid->I = YPid->ki * YPid->error;
        if (YPid->I < I_MIN) YPid->I = I_MIN;
        else if (YPid->I > I_MAX) YPid->I = I_MAX;
        //D term
        YPid->D = YPid->kd * YPid->error;
        if (YPid->D < D_MIN) YPid->D = D_MIN;
        else if (YPid->D > D_MAX) YPid->D = D_MAX;


    }



    return true;
}


