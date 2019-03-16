#ifndef BALLPHYSIC_H
#define BALLPHISIC_H

#include "../utils.h"
#include <immintrin.h>
#include <x86intrin.h>

#define     VEC             __m128i
#define     LOAD            _mm_loadu_si128
#define     STORE           _mm_storeu_si128

/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/
typedef struct Ball {
    bool        detected;
    uint16_t    x[8];
    uint16_t    y[8];
    uint16_t    dx;
    uint16_t    dy;
    uint16_t    fx;
    uint16_t    fy;
    float       v;
    float       phi;
} Ball;


Ball createBall(uint16_t _x, uint16_t _y);
void printBall(Ball b);
bool updateBall(Ball* b, uint16_t _x, uint16_t _y); 
void frameError(Ball* b);
void updatePosVec(Ball* b, uint16_t _x, uint16_t _y);
void updatePos(Ball* b, uint16_t _x, uint16_t _y);
void updateSpeed(Ball* b);
void frameError(Ball* b);
void updatePhase(Ball* b);
void updatePredictedPos(Ball* b);
float radiansToDegree(float radians);


#endif
