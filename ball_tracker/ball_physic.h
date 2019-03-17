#ifndef BALLPHYSIC_H
#define BALLPHISIC_H

#include "../utils.h"
#include <immintrin.h>
#include <emmintrin.h>
#include <x86intrin.h>

#define     VEC         __m128i
#define     LOAD        _mm_loadu_si128
#define     STORE       _mm_storeu_si128
#define     MUL         _mm_mulhi_epi16 

/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/
typedef struct Ball {
    bool        detected;
    uint16_t    x[8];
    uint16_t    y[8];
    short       dx[8];
    short       dy[8];
    short       smooth_dx;
    short       smooth_dy;
} Ball;

const float mask[7] = { 0.38, 0.25, 0.15, 0.10, 0.06, 0.04, 0.02 };

Ball createBall(uint16_t _x, uint16_t _y);
void printBall(Ball b);

void updateBall_VEC(Ball* b, uint16_t _x, uint16_t _y); 

void updatePosVec(Ball* b, uint16_t _x, uint16_t _y);
void updatePos(Ball* b, uint16_t _x, uint16_t _y);

void updateSpeed(Ball* b);
void updateSpeedVec(Ball* b);

//void frameError(Ball* b);
float radiansToDegree(float radians);


#endif
