/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/

#include <stdio.h>
#include <math.h>
#include "ball_physic.h"

/*allocate and initialize a Ball instance*/
Ball createBall(uint16_t _x, uint16_t _y){
	Ball b = {
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
	printf("\n  ===== Ball ================================= \n");
	

	printf("  ============================================= \n");
}


/*update position and compute new velocity and direction
	{this function is called in trackFilteredObject()}	*/
bool updateBall(Ball* b, uint16_t _x, uint16_t _y){
	updatePosVec(b, _x, _y);
	updateSpeed(b);
	updatePhase(b);
	updatePredictedPos(b);

	if (b->v==0) return false;
	else return true;
}
