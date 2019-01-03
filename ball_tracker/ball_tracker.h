#ifndef BALL_TRACKER_H
#define BALL_TRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "../utils.h"

using namespace cv;

//max number of objects to be detected in frame
#define     MAX_NUM_OBJECTS     16

//minimum and maximum object area
#define     MIN_OBJECT_AREA     20*20
#define     MAX_OBJECT_AREA     FRAME_HEIGHT*FRAME_WIDTH/1.5

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
void callBackFunc(int event, int x, int y, int flags, void* param);
void printHSV(mouseParams mp);

Ball* createBall(short _x, short _y);
void printBall(Ball* b, short global_clock);
inline bool updateBall(Ball* b, short _x, short _y);

#endif