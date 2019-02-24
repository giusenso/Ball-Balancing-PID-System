
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

//_ Data Structure _______________________

typedef struct Point_t{
    uint16_t    x;
    uint16_t    y;
} Point_t;

typedef struct ServoConfig {
    uint16_t     servoX;
    uint16_t     servoY;
} ServoConfig_t;


//_ Global Variables _______________________

#define     PI                  3.1415
#define     FRAME_WIDTH         640
#define     FRAME_HEIGHT        480
#define     TOLLERANCE          30
#define     SETPOINT_X          FRAME_WIDTH/2
#define     SETPOINT_Y          FRAME_HEIGHT/2

#define     bool        int
#define     false       0
#define     true        1


inline float radiansToDegree(float radians) {
    return radians * (180.0 / PI);
}


#endif
