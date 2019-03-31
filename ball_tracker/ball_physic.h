/**
 * @file ball_physic.h
 * @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
 * @brief Ball modelling and computation functions
 * @version 1.1
 * @date 2019-03-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef BALLPHYSIC_H
#define BALLPHISIC_H

#include "../utils.h"
#include <unistd.h>
#include <immintrin.h>
#include <emmintrin.h>

#define     VEC         __m128i
#define     LOAD        _mm_loadu_si128
#define     STORE       _mm_storeu_si128
#define     MUL         _mm_mulhi_epi16 

typedef struct Ball {
    bool        detected;
    uint16_t    x[8];
    uint16_t    y[8];
    short       dx[8];
    short       dy[8];
    short       smooth_dx;
    short       smooth_dy;
} Ball_t;


Ball_t createBall(uint16_t _x, uint16_t _y);

void printBall(Ball_t b);

void updateBall(Ball_t* b, uint16_t _x, uint16_t _y);

void updatePosVec(Ball_t* b, uint16_t _x, uint16_t _y);



#endif
