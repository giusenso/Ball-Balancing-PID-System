#include <stdlib.h>
#include <stdio.h>

#include "pid.h"

int main(){

    printf("\n# creating PID structs... ");
    PID_t* XPid;
    PID_t* YPid;
    printf("Done.\n");

//_ X PID SETUP __________________________________
    printf("\n# Setting up XPid parameters... ");
    setPid( XPid, SETPOINT_X, TOLLERANCE,
            100, // dt (in milliseconds)
            0.0 , 0.0, 0.0, // P, I, D
            5.0 , 2.0 , 1.0, // kp. ki, kd
            0.0, 0.0); // pre_error, error
        printf("Done.\n    Kp = %lf\n    Kd = %lf\n    Ki = %lf\n",
            XPid->kp, XPid->kd, XPid->ki);

//_ Y PID SETUP __________________________________
    printf("\n# Setting up YPid parameters... ");
    setPid( YPid, SETPOINT_Y, TOLLERANCE,
            0.1, // dt
            0.0 , 0.0, 0.0, // P, I, D
            5.0 , 2.0 , 1.0, // kp. ki, kd
            0.0, 0.0); // pre_error, error
        printf("Done.\n    Kp = %lf\n    Kd = %lf\n    Ki = %lf\n\n",
            YPid->kp, YPid->kd, YPid->ki);
//_______________________________________________


//_ X PID TEST __________________________________
    struct Point_t ball_pos;
    ball_pos.x = SETPOINT_X;
    ball_pos.y = SETPOINT_Y;
    
    int t = 0;
    for ( ; t<20 ; t++){ //ball not moving
        printf("\n\n*** timestamp: %d ***", t);
        PIDCompute(XPid, ball_pos.x);
        printPID(XPid);
    }

    for ( t=0 ; t<100 ; t++){
        ball_pos.x += t;
        printf("\n\n*** timestamp: %d ***\n    linear motion", t);
        PIDCompute(XPid, ball_pos.x);
        printPID(XPid);
    }

    for ( t=0 ; t<100 ; t++){
        ball_pos.x += -1.5*t;
        printf("\n\n*** timestamp: %d ***\n    acceleration=1.5", t);
        PIDCompute(XPid, ball_pos.x);
        printPID(XPid);
    }

    /*
    //testing random pattern movemen
    for (t=0 ; t<2500 ; t+=10){
        ball_pos.x += +1.5*t;
        printf("\n\n*** timestamp: %d ***\n    random pattern", t);
        PIDCompute(XPid, ball_pos.x);
        printPID(XPid);
    }
     
    TEST 1 FALED
    *** to be fixed ***
    1) ball over-react
    2) use vectorizzation (AVX/SSE)
    3) use pthread
    4) Servo implementation


    */
//_________________________________________



//__EXIT ROUTINE __________________________
    printf("\n# Free PID struct... ");
    free(XPid);
    free(YPid);
    printf("Done.\n");
    return 0;
}
