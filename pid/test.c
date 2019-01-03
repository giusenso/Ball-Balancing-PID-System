#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

#include "pid.h"

int main(){

    short ServoX = 0;
    Ball* ball = (Ball*)malloc(sizeof(Ball));
    ball->x[0] = 0;
    ball->y[0] = 0;
    ball->v = 0;
    ball->phy = 0;
   

    printf("\n# creating PID structs... ");
    PID_t* XPid = (PID_t*)malloc(sizeof(PID_t));
    PID_t* YPid = (PID_t*)malloc(sizeof(PID_t));
    printf("Done.\n");

    //_ X PID SETUP__________________________________
    printf("# Setting up XPid parameters... ");
    setPid( XPid, SETPOINT_X, TOLLERANCE,
            100 , // sample (in milliseconds)
            0.0 , 0.0, 0.0 , // P, I, D
            0.79 , 1.02 , 1.0); // kp. ki, kd
    printf("Done.\n    Kp = %lf\n    Ki = %lf\n    Kd = %lf\n\n",
            XPid->kp, XPid->ki, XPid->kd);


    //:::::::::::::: L O O P :::::::::::::::::::::::::::::::::::::::::::::::::::::
    int i = 0;
    printf("\n# Compute(): done.\n    ServoX     = %d\n    x_ball_pos = %d",
    ServoX, ball->x[0]);
    
    //faccio un giro a vuoto
    PIDCompute(XPid, ball->x[0], &ServoX);

    while (i<12){

        if (PIDCompute(XPid, ball->x[0], &ServoX)){
            printf("\n# Compute(): done.\n    ServoX     = %d\n    x_ball_pos = %d",
            ServoX, ball->x[0]);
            printPidState(XPid);
        }

        else{
            printf("# Compute(): none.\n    ServoX     = %d\n    x_ball_pos = %d",
            ServoX, ball->x[0]);
            printPidState(XPid);
        }

        ball->x[1] = ball->x[0];
        ball->x[0] += 6; 

        usleep(500000);
        i++;
    }


/*  //_ Y PID SETUP__________________________________
    printf("# Setting up YPid parameters... ");
    setPid( YPid, SETPOINT_Y, TOLLERANCE,
            100 , // sample
            0.0 , 0.0, 0.0 , // P, I, D
            5.0 , 2.0 , 1.0); // kp. ki, kd
    printf("Done.\n    Kp = %lf\n    Ki = %lf\n    Kd = %lf\n\n",
            YPid->kp, YPid->ki, YPid->kid);

    short ServoY = 0;
    x_ball_pos[0] = 260;
    x_ball_pos[1] = 94;

*/



//__EXIT ROUTINE __________________________
    printf("# Free PID struct... ");
    free(XPid);
    free(YPid);
    printf("Done.\n");
    return 0;
}