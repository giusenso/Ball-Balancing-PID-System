#ifndef BALL_PHYSIC_H
#define BALL_PHISIC_H

#include <stdint.h>
#include <immintrin.h>
#include "../utils.h"
#include "ball.h"

#define     VEC             __m128i
#define     LOAD            _mm_loadu_si128
#define     STORE           _mm_storeu_si128

/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/

Ball* createBall(uint16_t _x, uint16_t _y);
void printBall(Ball* b, uint16_t global_clock);
bool updateBall(Ball* b, uint16_t _x, uint16_t _y);

inline void updatePosVec(Ball* b, uint16_t _x, uint16_t _y){
    VEC v = LOAD((const VEC*)((b->x)-1));
    STORE((VEC*)(v), b->x);
    b->x[0] = _x;

    v = LOAD((const VEC*)((b->y)-1));
    STORE((VEC*)(v), b->y);
    b->y[0] = _y;

}

inline void updatePos(Ball* b, uint16_t _x, uint16_t _y){
    b->x[7] = b->x[6];
    b->x[6] = b->x[5];
    b->x[5] = b->x[4];
	b->x[4] = b->x[3];
	b->x[3] = b->x[2];
	b->x[2] = b->x[1];
	b->x[1] = b->x[0];
	b->x[0] = _x;

    b->y[7] = b->y[6];
    b->y[6] = b->y[5];
	b->y[5] = b->y[4];
	b->y[4] = b->y[3];
	b->y[3] = b->y[2];
	b->y[2] = b->y[1];
	b->y[1] = b->y[0];
	b->y[0] = _y;

}


inline void updateSpeed(Ball* b){
	b->dx = b->x[0] - b->x[1];	// x component
	b->dy = b->y[0] - b->y[1];	// y component
	b->v = fabsf(sqrt( pow(b->dx,2) + pow(b->dy,2) ));
}

inline void updatePhase(Ball* b){
	if(b->dx >= 1 || b->dx <= -1) b->phi = radiansToDegree(atan2(b->dy, b->dx));
}

inline void updatePredictedPos(Ball* b){
	b->fx = b->dx + b->x[0];
	b->fy = b->dy + b->y[0];
}


#endif
