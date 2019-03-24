
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>


//_ Data Structure _______________________

typedef struct Point_t{
    uint16_t    x;
    uint16_t    y;
} Point_t;

typedef struct ServoConfig {
    uint16_t     servoX;
    uint16_t     servoY;
} ServoConfig_t;

/*********************************************/
/*      SERVOS                               */
/*********************************************/

#define   PERIOD          1/F_CPU
#define   DEAD_BAND       3   *0.000001
#define   HALF_DEGREE     3.5 *0.000001

#define   X_HALF_ANGLE      24500
#define   Y_HALF_ANGLE      27800
#define   ANGLE_OFFSET      5500
#define	  X_MAX_ANGLE         X_HALF_ANGLE+ANGLE_OFFSET
#define   X_MIN_ANGLE         X_HALF_ANGLE-ANGLE_OFFSET
#define	  Y_MAX_ANGLE         Y_HALF_ANGLE+ANGLE_OFFSET
#define   Y_MIN_ANGLE         Y_HALF_ANGLE-ANGLE_OFFSET
/*********************************************/

//_ Global Variables _______________________

#define     PI                  3.1415
#define     CONTROL_AREA        400
#define     FRAME_WIDTH         640
#define     FRAME_HEIGHT        480
#define     TOLLERANCE          30
#define     SETPOINT_X          FRAME_WIDTH/2
#define     SETPOINT_Y          FRAME_HEIGHT/2
#define     FPS                 15
#define     MAX_SPEED 50    //max pixels change allowed in one frame

#define     bool        int
#define     false       0
#define     true        1



#endif
