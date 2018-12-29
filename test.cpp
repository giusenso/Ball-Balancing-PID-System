// Testing unit

#include <sstream>
#include <vector>
#include <string>
#include <iostream>
//#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

#include "serial_port/serial_port.h"
//#include <ball_tracking.h>	//to be implemented

using namespace cv;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const String windowName = "Original Image";
const String windowName1 = "HSV Image";
const String windowName2 = "Thresholded Image";
const String windowName3 = "After Morphological Operations";
const String trackbarWindowName = "Trackbars";

const Scalar RED = Scalar(0, 0, 254);
const Scalar GREEN = Scalar(0, 254, 0);
const Scalar BLUE = Scalar(254, 0, 0);

const char* serialPorts[5]= {	ttyACM0,
								ttyACM1,
								ttyACM2,
								ttyACM3,
								ttyACM4	
							};


void on_trackbar( int, void* )
{//This function gets called whenever a
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
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
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
void drawObjectV2(int x, int y, Mat &frame){

	//Draw version 2
	circle(frame,Point(x,y),35,GREEN, 1);

    line(frame,Point(x,0),Point(x,FRAME_HEIGHT),RED, 2);
	line(frame,Point(0,y),Point(FRAME_WIDTH,y),BLUE,2);

	putText(frame,intToString(y),Point(x+2,y-42),1,1,BLUE,2);
	putText(frame,intToString(x),Point(x+40,y+10),1,1,RED,2);

}

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	


}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
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

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObjectV2(x,y,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}


//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//********** M A I N **********************************************************************************
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int main(int argc, char* argv[]){

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;

	//matrix storage for HSV image
	Mat HSV;

	//matrix storage for binary threshold image
	Mat threshold;

	//x and y values for the location of the object
	int x=0, y=0;

	//create slider bars for HSV filtering
	createTrackbars();


	//initialize camera__________________________
	VideoCapture capture;

	int CAM_NUMBER = 0;
	for ( ; CAM_NUMBER<3 ; CAM_NUMBER++){
		capture.open(CAM_NUMBER);
		if (capture.isOpened()){
			printf("/dev/video%d successfully opened\n", CAM_NUMBER);
			break;
		}
	}
	if (CAM_NUMBER == 3){
		perror("ERROR: NO dev/video* DEVICE CONNECTED");
		return -1;
	}

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	
	//initialize serial communication______________
	int fd = -1;
	for (int k=0 ; k<5 ; k++){
		fd = open(serialPorts[k], O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd >= 0){
			printf("# %s successfully opened\n", serialPorts[k]);
			break;
		}
	}
	if (fd < 0){
		printf("\nERROR: no serial device avaible!\n");
		return -1;
	}
	
	setSerialAttributes(fd);
	
	//_____________________________________________

	//initialize write_buffer_______________________
	uint8_t* write_buffer = (uint8_t*)malloc(sizeof(uint8_t)*sizeof(ServoConfig_t));
	int bytes_written = 0;
	printf("# write_buffer allocated\n");

	//Initialize data structure________________
	ServoConfig_t* config = (ServoConfig_t*) malloc(sizeof(ServoConfig_t));
		config->ServoX =	 0xFF;
		config->ServoY =	 0xFF;
		config->flag1  =	 0xFF;
		config->flag2  =	 0xF0;
	
	printServoConfig(config);
	printf("\n### All parameters setted, ready to go...\n\n");

	usleep(2000000); //for debug
	//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	while(true){
		//store image to matrix
		capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		trackFilteredObject(x,y,threshold,cameraFeed);

		/*------------------------------- Write data to serial port -----------------------------*/

		encodeConfig(config, write_buffer);
		printf("encode result: %s\n", write_buffer);

		bytes_written = write(fd, write_buffer, sizeof(ServoConfig_t));
		//printf("\n===> [%s] written to ttyACM0 \n", write_buffer);
		printServoConfig(config);
		printf("===> %d Bytes written to ttyACM0\n\n", bytes_written);

		/*--------------------------------------------------------------------------------------*/

		//show frames 
		imshow(windowName2,threshold);
		imshow(windowName,cameraFeed);
		//imshow(windowName1,HSV);
		
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		if(waitKey(30) >= 0) break;


		config->ServoX++;
		config->ServoY++;
		usleep(1000000);

	}//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	close(fd);
	free(config);
	free(write_buffer);

	return 0;
}