#include <stdlib.h>
#include <stdio.h>

#include "pid.h"

int main(){

    printf("\n# creating PID structs... ");
    PID_t* XPid = (PID_t*)malloc(sizeof(PID_t));
    PID_t* YPid = (PID_t*)malloc(sizeof(PID_t));
    printf("Done.\n");

    //_ X PID __________________________________
    printf("# Setting up XPid parameters... ");
    setPid( XPid, SETPOINT_X, TOLLERANCE,
            100, // dt (in milliseconds)
            0.0 , 0.0, 0.0, // P, I, D
            5.0 , 2.0 , 1.0, // kp. ki, kd
            0.0, 0.0); // pre_error, error
    printf("Done.\n    Kp = %lf\n    Kd = %lf\n    Ki = %lf\n\n",
            XPid->kp, XPid->kd, XPid->ki);

    //_ Y PID __________________________________
    printf("# Setting up YPid parameters... ");
    setPid( YPid, SETPOINT_Y, TOLLERANCE,
            0.1, // dt
            0.0 , 0.0, 0.0, // P, I, D
            5.0 , 2.0 , 1.0, // kp. ki, kd
            0.0, 0.0); // pre_error, error
    printf("Done.\n    Kp = %lf\n    Kd = %lf\n    Ki = %lf\n\n",
            YPid->kp, YPid->kd, YPid->ki);


    //_ X PID __________________________________
    












//__EXIT ROUTINE __________________________
    printf("# Free PID struct... ");
    free(XPid);
    free(YPid);
    printf("Done.\n");
    return 0;
}