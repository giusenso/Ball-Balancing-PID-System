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
		.dx 	= { 0,0,0,0,0,0,0,0 },
		.dy 	= { 0,0,0,0,0,0,0,0 },
		.smooth_dx 	= 0,
		.smooth_dy 	= 0
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


////////////////////////////////////////////////////////////////
/// ARRAY VERSION //////////////////////////////////////////////

void updateBall(Ball* b, uint16_t _x, uint16_t _y){
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

	//update X filtered speed
	b->smooth_dx =
	   (b->dx[0]+
		b->dx[1]+
		b->dx[2]+
		b->dx[3]+
		b->dx[4])/5;
	
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
	
	//update Y filtered speed
	b->smooth_dy =
	   (b->dy[0]+
		b->dy[1]+
		b->dy[2]+
		b->dy[3]+
		b->dy[4])/5;
}



/////////////////////////////////////////////////////////////////
/// VECTORIAL VERSION ///////////////////////////////////////////

/*	 update position and compute new speed
	{this function is called in trackFilteredObject()}	*/
void updateBall_VEC(Ball* b, uint16_t _x, uint16_t _y){

	//update x pos //////////////////////////////////////////////
	VEC vpos = LOAD( (const VEC*)((b->x)-1) );
    STORE( (VEC*)(b->x), vpos);
    b->x[0] = _x;

	//update x speed ////////////////////////////////////////////
	VEC vspeed = LOAD( (const VEC*)((b->dx)-1) );
    STORE( (VEC*)(b->dx), vspeed);
	b->dx[0] = b->x[0] - b->x[1];

	b->smooth_dx =
		b->dx[0] * mask[0] +
		b->dx[1] * mask[1] +
		b->dx[2] * mask[2] +
		b->dx[3] * mask[3] +
		b->dx[4] * mask[4] +
		b->dx[5] * mask[5] +
		b->dx[6] * mask[6];


	//update y pos ////////////////////////////////////////////
    vpos = LOAD((const VEC*)((b->y)-1));
    STORE( (VEC*)(b->y), vpos );
    b->y[0] = _y;

	//update y speed //////////////////////////////////////////
    vspeed = LOAD( (const VEC*)((b->dy)-1) );
    STORE( (VEC*)(b->dy), vspeed );
	b->dy[0] = b->y[0] - b->y[1];

	b->smooth_dy =
		b->dy[0] * mask[0] +
		b->dy[1] * mask[1] +
		b->dy[2] * mask[2] +
		b->dy[3] * mask[3] +
		b->dy[4] * mask[4] +
		b->dy[5] * mask[5] +
		b->dy[6] * mask[6];
}


inline float radiansToDegree(float radians) {
    return radians * (180.0 / PI);
}
