#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ball_tracker.h"

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

const String windowName = "Original Image";
const String windowName1 = "HSV Image";
const String windowName2 = "Thresholded Image";
const String windowName3 = "After Morphological Operations";
const String trackbarWindowName = "Trackbars";


/*************************************************************************************
 *--------  drawing functions -------------------------------------------------------*
 *************************************************************************************/

void on_trackbar( int, void* ){
	//This function gets called whenever a
	// trackbar position is changed
}

String intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(){

	//create window for trackbars
    namedWindow(trackbarWindowName,0);

	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN");
	sprintf( TrackbarName, "H_MAX");
	sprintf( TrackbarName, "S_MIN");
	sprintf( TrackbarName, "S_MAX");
	sprintf( TrackbarName, "V_MIN");
	sprintf( TrackbarName, "V_MAX");
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


}

void drawObjectV1(int x, int y, Mat &frame){

	//Draw version 1

	circle(frame,Point(x,y),24,Scalar(0,255,0),3);
	//vertical line
    if(y-25>0)
    line(frame,Point(x,0),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);

	//orizzontal line
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

void drawObjectV2(Ball* b, Mat &frame){

	//_ Draw version 2 ____________________________

	//draw setpoint area
	circle(frame, Point(SETPOINT_X,SETPOINT_Y), 2, DARK_GREEN, 3);
	rectangle(	frame,
				Point(SETPOINT_X-TOLLERANCE, SETPOINT_Y-TOLLERANCE),
				Point(SETPOINT_X+TOLLERANCE, SETPOINT_Y+TOLLERANCE),
				DARK_GREEN, 2, LINE_8, 0);


	//draw ball lines for position spot
	rectangle(	frame,
				Point(b->x[0]-30, b->y[0]-30),
				Point(b->x[0]+30, b->y[0]+30),
				GREEN, 2, LINE_8, 0);

    line(frame, Point(b->x[0], 0), Point(b->x[0], FRAME_HEIGHT), BLUE, 1);
	line(frame, Point(0, b->y[0]), Point(FRAME_WIDTH, b->y[0]), BLUE, 1);


	//draw velocity arrow 
	if (b->v > 2.0){
		arrowedLine(frame, Point(b->x[0], b->y[0]), Point(b->fx, b->fy) , RED, 2, 8, 0 , 0.4);
	}
	//draw previous positions	
	for (int i=1 ; i<6 ; i++){
		circle( frame, Point(b->x[i], b->y[i]), 2, ORANGE, -1, 8, 0 );
	}

	//draw square area
	rectangle(	frame, 
				Point(SETPOINT_X-225, SETPOINT_Y-225),
				Point(SETPOINT_X+225, SETPOINT_Y+225), 
				ORANGE, 3, LINE_8, 0);

	//display ball info
	putText(frame, "BALL DETECTED", Point(100,32), 1, 1, GREEN, 2);
	putText(frame,intToString(b->y[0]),Point(b->x[0]+2,b->y[0]-42),1,1,BLUE,2);
	putText(frame,intToString(b->x[0]),Point(b->x[0]+40,b->y[0]+14),1,1,BLUE,2);


}

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.

	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));

    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(5,5));

	//Mat 

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	
	morphologyEx(thresh, thresh, MORPH_CLOSE, Mat::ones(5, 5, CV_8U));

	//release
	erodeElement.release();
	dilateElement.release();

}

void trackFilteredObject(Ball* b, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	//temp =threshold(buildBox(b));

	
	//these two vectors needed for output of findContours
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					objectFound = true;
					circleDetector(cameraFeed, threshold);
					updateBall(b, moment.m10/area, moment.m01/area);
					refArea = area;
				}
				else{
					objectFound = false;
					putText(cameraFeed, "BALL NOT FOUND", Point(100,32), 1, 1, RED , 2);
				}
			}

			//let user know you found an object
			if(objectFound ==true){
				drawObjectV2( b , cameraFeed );
			}
			
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(100,32),1,2,RED,2);
	}
	//release memory
	temp.release();
}

void circleDetector(Mat cameraFeed, Mat threshold){

	Mat gray;
	threshold.copyTo(gray);
	//cvtColor(threshold, gray, CV_BGR2GRAY);
	GaussianBlur( gray, gray, Size(7, 7), 1.8, 1.8 );

	std::vector<Vec3f> circles;
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 35, 100, 25, 10, 45);
	
	for(size_t i = 0 ; i < circles.size() ; i++){
		printf("%d    radius: %.1f \n", i+1, circles[i][2]);
		
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		short radius = cvRound(circles[i][2]);

		circle( cameraFeed, center, 3, ORANGE, -1, 8, 0 );     // circle center
		circle( cameraFeed, center, radius+1, GREEN, 2, 8, 0 );  // circle outline
  	}
	gray.release();
}


//create square box and find for ball only here
void createSquareBox(Ball* b, Mat cameraFeed, Mat threshold){			
	//delete from threshol all out of box point
	//box is centered in (fx, fy)

}

void callBackFunc(int event, int x, int y, int flags, void* param){
    if  ( event == EVENT_LBUTTONUP ){
        printf("\n# MOUSE CLICK EVENT\n	x: %d\n	y: %d\n", x, y);

		// Mount back the parameters
    	mouseParams* mp = (mouseParams*)param;
		Mat HSV;
		cvtColor(mp->_mat, HSV, COLOR_BGR2HSV);
		Vec3b hsv = HSV.at<Vec3b>(x,y);

		int h = ((int)(hsv.val[0]))*(250.0/360.0);
		int s = (int)(hsv.val[1]);
		int v = (int)(hsv.val[2]);

		int epsilon = 9;

		*mp->_H_MIN = (h-epsilon);
		if (*mp->_H_MIN < 0)*mp->_H_MIN = 0;

		*mp->_H_MAX = (h+epsilon);
		if (*mp->_H_MAX > 255)*mp->_H_MAX = 255;

		*mp->_S_MIN = (126);
		if (*mp->_S_MIN < 0)*mp->_S_MIN = 0;

		*mp->_S_MAX = (255);
		if (*mp->_S_MAX > 255)*mp->_S_MAX = 255;

		*mp->_V_MIN = (60);
		if (*mp->_V_MIN < 0)*mp->_V_MIN = 0;

		*mp->_V_MAX = (245);
		if (*mp->_V_MAX > 255)*mp->_V_MAX = 255;

		printHSV(*mp);
    }
}

void printHSV(mouseParams mp){
	printf("\n  =======================\n");
	printf(" ||	H MIN = %d	||\n", *mp._H_MIN);
	printf(" ||	  MAX = %d	||\n", *mp._H_MAX);
	printf(" ||	S MIN = %d	||\n", *mp._S_MIN);
	printf(" ||	  MAX = %d	||\n", *mp._S_MAX);
	printf(" ||	V MIN = %d	||\n", *mp._V_MIN);
	printf(" ||	  MAX = %d	||\n", *mp._V_MAX);
	printf("  =======================\n");
}



/*************************************************************************************
 *------- Ball physics modelling and computation functions --------------------------*
 *************************************************************************************/

/*allocate and initialize a Ball instance*/
Ball* createBall(short _x, short _y){
    Ball* b = (Ball*)malloc(sizeof(Ball));
    b->x[0] = b->x[1] = b->x[2] = b->x[3] = b->x[4] = b->x[5] = _x;
    b->y[0] = b->y[1] = b->y[2] = b->y[3] = b->y[4] = b->y[5] = _y;
	b->dx = 0;
	b->dy = 0;
	b->fx = _x;
	b->fy = _y;
    b->v = 0;
	b->phi = 0;
	return b;
}

/*print Ball instance function for debugging*/
void printBall(Ball* b, short global_clock){
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
bool updateBall(Ball* b, short _x, short _y){
	
	updatePos(b, _x, _y);
	updateSpeed(b);
	updatePhase(b);
	updatePredictedPos(b);
	
	if (b->dx==0 && b->dy==0) return false;
	else return true;
	
}

