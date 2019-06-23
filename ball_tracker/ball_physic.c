/**
 * @file ball_physic.c
 * @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
 * @brief Ball modelling and computation functions
 * @version 1.1
 * @date 2019-03-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#include "ball_physic.h"

//size of the moving average
#define 	N	3


/**
 * @brief Create a Ball object
 * 
 * @param _x coordinate in pixels
 * @param _y coordinate in pixels
 * @return Ball_t 
 */
Ball_t createBall(uint16_t _x, uint16_t _y){
	Ball_t b = {
		.detected = false,
		.x 		= { _x,_x,_x,_x,_x,_x,_x,_x },
		.y 		= { _y,_y,_y,_y,_y,_y,_y,_y },
		.dx 	= { 0,0,0,0,0,0,0,0 },
		.dy 	= { 0,0,0,0,0,0,0,0 },
		.smooth_dx 	= 0,
		.smooth_dy 	= 0
	};
	return b;
}


/**
 * @brief update position, speed and moving average.
 * [optimized by compiler]
 * @param b Ball_t object
 * @param _x new x coordinate in pixels
 * @param _y new y coordinate in pixels
 */
void updateBall(Ball_t* b, uint16_t _x, uint16_t _y){
	//update X position
	b->x[7] = b->x[6];
    b->x[6] = b->x[5];
    b->x[5] = b->x[4];
	b->x[4] = b->x[3];
	b->x[3] = b->x[2];
	b->x[2] = b->x[1];
	b->x[1] = b->x[0];
	b->x[0] = _x;

	//update X speed
	b->dx[7] = b->dx[6];
	b->dx[6] = b->dx[5];
	b->dx[5] = b->dx[4];
	b->dx[4] = b->dx[3];
	b->dx[3] = b->dx[2];
	b->dx[2] = b->dx[1];
	b->dx[1] = b->dx[0];
	b->dx[0] = b->x[0] - b->x[1];

	//recursively update moving average for X
	b->smooth_dx =
		(N * b->smooth_dx - b->dx[N] + b->dx[0]) / N;

	
	//update Y position
	b->y[7] = b->y[6];
	b->y[6] = b->y[5];
	b->y[5] = b->y[4];
	b->y[4] = b->y[3];
	b->y[3] = b->y[2];
	b->y[2] = b->y[1];
	b->y[1] = b->y[0];
	b->y[0] = _y;
	
	//update Y speed
	b->dy[7] = b->dy[6];
	b->dy[6] = b->dy[5];
	b->dy[5] = b->dy[4];
	b->dy[4] = b->dy[3];
	b->dy[3] = b->dy[2];
	b->dy[2] = b->dy[1];
	b->dy[1] = b->dy[0];
	b->dy[0] = b->y[0] - b->y[1];
	
	//recursively update moving average
	b->smooth_dy =
		(N * (b->smooth_dy) - b->dy[N] + b->dy[0]) / N;
}


/**
 * @brief Vectorial version of updateBall()
 * just for testing purpouse.
 * this version use instrinsics for better performace.
 * note: gcc with -o2 flags or superior do it automatically
 * and better.
 * 
 * 
 * @param b 
 * @param _x 
 * @param _y 
 */
void updateBallVec(Ball_t* b, uint16_t _x, uint16_t _y){

	//update x pos
	VEC vpos = LOAD( (const VEC*)((b->x)-1) );
    STORE( (VEC*)(b->x), vpos);
    b->x[0] = _x;

	//update x speed
	VEC vspeed = LOAD( (const VEC*)((b->dx)-1) );
    STORE( (VEC*)(b->dx), vspeed);
	b->dx[0] = b->x[0] - b->x[1];

	b->smooth_dx =
	   (b->dx[0]+
		b->dx[1]+
		b->dx[2]+
		b->dx[3]+
		b->dx[4])/5;


	//update y pos
    vpos = LOAD((const VEC*)((b->y)-1));
    STORE( (VEC*)(b->y), vpos );
    b->y[0] = _y;

	//update y speed 
    vspeed = LOAD( (const VEC*)((b->dy)-1) );
    STORE( (VEC*)(b->dy), vspeed );
	b->dy[0] = b->y[0] - b->y[1];

	b->smooth_dy =
	   (b->dy[0]+
		b->dy[1]+
		b->dy[2]+
		b->dy[3]+
		b->dy[4])/5;
}


/**
 * @brief Debug print
 * 
 * @param b Ball_t object
 */
void printBall(Ball_t b){
	printf("\n    ===== Ball ===================== \n");
	if(b.detected) 	printf("   ||  detect:  yes\n");
	else 			printf("   ||  detect:  no\n");
	printf("   ||  x:	%d\n", b.x[0]);
	printf("   ||  y:	%d\n", b.y[0]);
	printf("    ================================ \n");
}
