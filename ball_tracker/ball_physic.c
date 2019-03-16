/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/

#include "ball_physic.h"

/*allocate and initialize a Ball instance*/
Ball createBall(uint16_t _x, uint16_t _y){
	Ball b = {
		.detected = false,
		.x 		= { _x,_x,_x,_x,_x,_x,_x,_x },
		.y 		= { _y,_y,_y,_y,_y,_y,_y,_y },
		.dx 	= 0,
		.dy 	= 0,
		.fx 	= 0,
		.fy 	= 0,
		.v 		= 0,
		.phi 	= 0
	};
	return b;
}

/*print Ball instance function for debugging*/
void printBall(Ball b){
	printf("\n    ===== Ball ===================== \n");
	if(b.detected) 	printf("   ||  detect:  yes\n");
	else 			printf("   ||  detect:  no\n");
	printf("   ||  x:	%d\n", b.x[0]);
	printf("   ||  y:	%d\n", b.y[0]);
	printf("    ================================ \n");
}

inline void updatePosVec(Ball* b, uint16_t _x, uint16_t _y){
    __m128i v = _mm_loadu_si128( (const __m128i*)((b->x)-1) );
    _mm_storeu_si128( (__m128i*)(b->x), v);
    b->x[0] = _x;

    v = _mm_loadu_si128((const __m128i*)((b->y)-1));
    _mm_storeu_si128( (__m128i*)(b->y), v);
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

//error procedure
inline void frameError(Ball* b){
	b->x[0] = b->fx;
	b->y[0] = b->fy;
	updateSpeed(b);
}

inline void updatePhase(Ball* b){
	if(b->dx >= 1 || b->dx <= -1) b->phi = radiansToDegree(atan2(b->dy, b->dx));
}

inline void updatePredictedPos(Ball* b){
	b->fx = b->dx + b->x[0];
	b->fy = b->dy + b->y[0];
}

/*update position and compute new velocity and direction
	{this function is called in trackFilteredObject()}	*/
bool updateBall(Ball* b, uint16_t _x, uint16_t _y){
	updatePosVec(b, _x, _y);
	updateSpeed(b);
	//if(b->v > MAX_SPEED) frameError(b);
	updatePhase(b);
	updatePredictedPos(b);

	if (b->v==0) return false;
	else return true;
}

inline float radiansToDegree(float radians) {
    return radians * (180.0 / PI);
}
