#ifndef BALL_TRACKER_H
#define BALL_TRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "../utils.h"

using namespace cv;

#define 	BOX_SIZE			200
//max number of objects to be detected in frame
#define     MAX_NUM_OBJECTS     20

//minimum and maximum object area
#define     MIN_OBJECT_AREA     14*14
#define     MAX_OBJECT_AREA     FRAME_HEIGHT*FRAME_WIDTH/5

//Colors
const Scalar RED = Scalar(0, 0, 251);
const Scalar GREEN = Scalar(0, 254, 0);
const Scalar BLUE = Scalar(254, 0, 0);
const Scalar ORANGE = Scalar(0, 165, 254);
const Scalar DARK_GREEN = Scalar(35,149,22);

//initial min and max HSV filter values.
//these will be changed using trackbars
extern int H_MIN;
extern int H_MAX;
extern int S_MIN;
extern int S_MAX;
extern int V_MIN;
extern int V_MAX;

//names that will appear at the top of each window
extern const String windowName;
extern const String windowName1;
extern const String windowName2;
extern const String windowName3;
extern const String trackbarWindowName;


typedef struct mouseParams{
	Mat _mat;
	int* _H_MIN;
	int* _H_MAX;
	int* _S_MIN;
	int* _S_MAX;
	int* _V_MIN;
	int* _V_MAX;
}mouseParams_t;
//_ Function Signature _________________________

void on_trackbar( int, void* );
String intToString(int number);
void createTrackbars();
void drawObjectV1(int x, int y, Mat &frame);
void drawObjectV2(Ball* b, Mat &frame);
void morphOps(Mat &thresh);
void trackFilteredObject(Ball* b, Mat threshold, Mat &cameraFeed);
void circleDetector(Mat cameraFeed, Mat threshold);
void createSquareBox(Mat cameraFeed, Mat threshold);
void callBackFunc(int event, int x, int y, int flags, void* param);
void printHSV(mouseParams mp);

Ball* createBall(short _x, short _y);
void printBall(Ball* b, short global_clock);
bool updateBall(Ball* b, short _x, short _y);



inline void updatePos(Ball* b, short _x, short _y){
	b->x[5] = b->x[4];
	b->x[4] = b->x[3];
	b->x[3] = b->x[2];
	b->x[2] = b->x[1];
	b->x[1] = b->x[0];
	b->x[0] = _x;
	
	b->y[5] = b->y[4];
	b->y[4] = b->y[3];
	b->y[3] = b->y[2];
	b->y[2] = b->y[1];
	b->y[1] = b->y[0];
	b->y[0] = _y;
	
	b->dx = b->x[0] - b->x[1];
	b->dy = b->y[0] - b->y[1];	
}

inline void updateSpeed(Ball* b){
	b->v = fabsf(sqrt( pow(b->dx,2) + pow(b->dy,2) ));
}

inline void updatePhase(Ball* b){
	if(b->dx >= 1 || b->dx <= -1) b->phi = radiansToDegree(atan2(b->dy, b->dx));
}

inline void updatePredictedPos(Ball* b){
	b->fx = b->dx + b->x[0];
	b->fy = b->dy + b->y[0];
}

inline Rect buildBox(Ball* b){

    //prepare top left corner of the box
	short tl_x = b->x[0] - BOX_SIZE/2;
    if (tl_x >= FRAME_WIDTH-BOX_SIZE) tl_x = 
        FRAME_WIDTH-BOX_SIZE;
    if(tl_x <=0) tl_x = 0;

	short tl_y = b->y[0] - BOX_SIZE/2;
    if (tl_y >= FRAME_HEIGHT-BOX_SIZE) tl_y = 
        FRAME_HEIGHT-BOX_SIZE;
    if(tl_y <=0) tl_y = 0;

    printf("\nBOX\ntx %d\nty %d\nbx %d\nby %d\n", 
      tl_x, tl_y, tl_x+BOX_SIZE, tl_y+BOX_SIZE);
    return Rect(tl_x, tl_y, tl_x+BOX_SIZE, tl_y+BOX_SIZE);
}


#endif