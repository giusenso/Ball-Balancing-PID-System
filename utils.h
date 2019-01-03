
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

//_ Data Structure _______________________


typedef struct ServoConfig {
    uint8_t     ServoX;
    uint8_t     ServoY;
    uint8_t     flag1;
    uint8_t     flag2;
} ServoConfig_t;


typedef struct Ball {
    short x[6]; //store up to 2^5 frame ago
    short y[6];
    float v;    //velocity
    float phi;  //direction
} Ball;




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
