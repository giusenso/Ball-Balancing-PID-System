/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/

#include <stdio.h>
#include <math.h>
#include "ball_physic.h"

/*allocate and initialize a Ball instance*/
Ball* createBall(uint16_t _x, uint16_t _y){
    Ball* b;
    b->x[0] = b->x[1] = b->x[2] = b->x[3] = b->x[4] = b->x[5] = b->x[6] = b->x[7] = _x;
    b->y[0] = b->y[1] = b->y[2] = b->y[3] = b->y[4] = b->y[5] = b->y[6] = b->y[7] = _y;
	b->dx = 0;
	b->dy = 0;
	b->fx = _x;
	b->fy = _y;
    b->v = 0;
	b->phi = 0;
	return b;
}

/*print Ball instance function for debugging*/
void printBall(Ball* b, uint16_t global_clock){
	printf("\n  ===== Ball (%d) ================================= \n", global_clock);
	printf(" ||	x:	[ %d , %d , %d , %d , %d , %d ]	\n",
		b->x[0],b->x[1],b->x[2],b->x[3],b->x[4],b->x[5]);
	printf(" ||	y:	[ %d , %d , %d , %d , %d , %d ]	\n",
		b->y[0],b->y[1],b->y[2],b->y[3],b->y[4],b->y[5]);
	printf(" ||	dx:	  %d	   		\n", b->dx);
	printf(" ||	dy:	  %d   			\n", b->dy);
	printf(" ||	fx:	  %d	   		\n", b->fx);
	printf(" ||	fy:	  %d   			\n", b->fy);
	printf(" ||	v:	  %.1f		   			\n", b->v);
	printf(" ||	phy:	  %.1f  				\n", b->phi);
	printf("  ================================================= \n");
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
