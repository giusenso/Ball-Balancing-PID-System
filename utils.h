
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>


typedef struct ServoConfig {
    uint8_t     ServoX;
    uint8_t     ServoY;
    uint8_t     flag1;
    uint8_t     flag2;
} ServoConfig_t;


typedef struct Point_t {
    short   x;
    short   y;
}Point_t;


//_ Global Variables Here _______________________

#define     FRAME_WIDTH         640
#define     FRAME_HEIGHT        480
#define     TOLLERANCE          30
#define     SETPOINT_X                FRAME_WIDTH/2
#define     SETPOINT_Y                FRAME_HEIGHT/2

#define     bool        int
#define     false       0
#define     true        1


#endif
