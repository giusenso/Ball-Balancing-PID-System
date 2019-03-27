#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>

//_ Data Structure _______________________
#define     bool        int
#define     false       0
#define     true        1

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
#define   X_HALF_ANGLE      23100
#define   Y_HALF_ANGLE      24600
#define   ANGLE_OFFSET      5800
#define	  X_MAX_ANGLE       X_HALF_ANGLE+ANGLE_OFFSET
#define   X_MIN_ANGLE       X_HALF_ANGLE-ANGLE_OFFSET
#define	  Y_MAX_ANGLE       Y_HALF_ANGLE+ANGLE_OFFSET
#define   Y_MIN_ANGLE       Y_HALF_ANGLE-ANGLE_OFFSET
/*********************************************/

//_ Global Variables _______________________

#define     PI                  3.1415
#define     CONTROL_AREA        348
#define     FRAME_WIDTH         640
#define     FRAME_HEIGHT        480
#define     SETPOINT_X          FRAME_WIDTH/2
#define     SETPOINT_Y          FRAME_HEIGHT/2




#endif
