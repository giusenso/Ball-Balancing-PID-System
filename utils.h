/**
 * @file utils.h
 * @author Giuseppe Sensolini
 * 
 * @brief provide basic data structures and global variables
 * @date 2019-01-29
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#define     bool        int
#define     false       0
#define     true        1

//_ Data Structure _______________________

typedef struct ServoConfig {
    uint16_t     xPulse;
    uint16_t     yPulse;
} ServoConfig_t;


//_ Global Variables _______________________

/*========== about servos =======================*/
#define   ANGLE_OFFSET      5900

#define   X_HALF_ANGLE      23650
#define	  X_MAX_ANGLE       X_HALF_ANGLE+ANGLE_OFFSET
#define   X_MIN_ANGLE       X_HALF_ANGLE-ANGLE_OFFSET

#define   Y_HALF_ANGLE      24850
#define	  Y_MAX_ANGLE       Y_HALF_ANGLE+ANGLE_OFFSET
#define   Y_MIN_ANGLE       Y_HALF_ANGLE-ANGLE_OFFSET

/*===============================================*/
#define     PI                  3.1415

#define     CONTROL_AREA        350

#define     FRAME_WIDTH         640
#define     FRAME_HEIGHT        480

#define     SETPOINT_X          FRAME_WIDTH/2
#define     SETPOINT_Y          FRAME_HEIGHT/2
/*===============================================*/


#endif
