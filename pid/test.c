#include <stdlib.h>
#include <stdio.h>

#include "pid.h"

int main(){

    printf("\n# creating PID structs... ");
    PID_t* XPID;
    printf("Done.\n");

//_ X PID SETUP __________________________________
    printf("\n# Setting up XPID parameters... ");
    
    setPid( XPID, SETPOINT_X,
            ANGLE_OFFSET/CONTROL_AREA, 0, 0,
            0, 0, FRAME_RATE,
            MIN_ANGLE, MAX_ANGLE);
    
    printf("Done.\n    Kp = %lf\n    Kd = %lf\n    Ki = %lf\n",
            XPID->Kp, XPID->Kd, XPID->Ki);



    return 0;
}
