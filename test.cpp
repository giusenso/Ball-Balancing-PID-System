// Testing unit

#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

#include "serial_port/serial_port.h"
#include "ball_tracker/ball.h"
#include "ball_tracker/ball_physic.h"
#include "ball_tracker/ball_tracker.h"
#include "pid/pid.h"

using namespace cv;


//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//********** M A I N **********************************************************************************
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int main(int argc, char* argv[]){

	// Mat Array = [ webcam | masked | HSV ]
	Mat MATS[3];

	//create slider bars for HSV filtering
	createTrackbars();

	//initialize camera__________________________
	VideoCapture capture;

	//enable this to capture mp4 file
	//capture.open("/home/jius/Desktop/ball-tracking-platform/ball_tracker/samples/test3.mp4");


	int CAM_NUMBER = 0;
	for ( ; CAM_NUMBER<3 ; CAM_NUMBER++){
		capture.open(CAM_NUMBER);
		if (capture.isOpened()){
			printf("# /dev/video%d successfully opened\n", CAM_NUMBER);
			break;
		}
	}
	if (CAM_NUMBER == 3){
		perror("\nERROR: NO dev/video* DEVICE CONNECTED\n");
		return -1;
	}

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);


	//initialize serial communication______________
	int fd = -1;
	int device_opened = openSerialCommunication(&fd);
	if(device_opened >= 0){
		setSerialAttributes(fd);
		printf("# %s successfully opened\n", serialPorts[device_opened]);
	}
	else{
		printf("\nERROR: no serial device avaible!\n");
		return -1;
	}

	//_____________________________________________

	/*
	//grab 1 frame to capture hsv ball values

	imshow(windowName2, MATS[1]);
	imshow(windowName, MATS[0]);

	mouseParams_t mp;
	mp._mat = MATS[0];
	mp._H_MIN = &H_MIN;
	mp._H_MAX = &H_MAX;
	mp._S_MIN = &S_MIN;
	mp._S_MAX = &S_MAX;
	mp._V_MIN = &V_MIN;
	mp._V_MAX = &V_MAX;
	setMouseCallback( windowName, callBackFunc, (void*)&mp);
	printHSV(mp);
	*/
//________________________________________________

	//initialize write_buffer_______________________
	uint8_t buf[5] = { 0,0, 0,0, '\n'};
	int bytes_written = 0;
	printf("# write_buffer allocated\n");

	//create ball instance
	Ball ball = createBall(0, 0);
	//printBall(ball);
	
//_ X PID SETUP __________________________________
	printf("\n# creating PID structs... ");
    
    PID_t XPID = createPID(150, 0, 0);
    printPID(XPID);
//________________________________________________
	//Initialize data structure________________
	ServoConfig_t config = { 
		.servoX = 0xFFFF,
		.servoY = 0xFFFF
	};
	printServoConfig(&config);

	/*
	namedWindow("A", WINDOW_AUTOSIZE);
	namedWindow("B", WINDOW_AUTOSIZE);
	Mat OUTPUT;
	Mat A = imread("/home/jius/Desktop/A.jpg", IMREAD_COLOR);
	Mat B = imread("/home/jius/Desktop/B.jpg", IMREAD_COLOR);
	A.convertTo(A, CV_64F);
	B.convertTo(A, CV_64F);
	//cv::hconcat(A, B, OUTPUT); // Accept array + size
	//assert(480 == OUTPUT.rows && 1280 == OUTPUT.cols);
	imshow("A",  A);
	imshow("B",  B);
	//imshow(windowName2, OUTPUT);
	*/

	printf("OKI");
	usleep(2000000); //for debug

	int global_clock = 0; //global clock
	float fps = FPS;
	time_t start, end;
	time(&start);

	printf("\n### All parameters setted, ready to go...\n\n");

	//::: MAIN LOOP :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	while(true){

		//store image to matrix
		capture.read(MATS[0]);
		//fps = capture.get(CV_CAP_PROP_FPS);

		//convert frame from BGR to MATS[2] colorspace
		cvtColor(MATS[0],MATS[2],COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//MATS[1] matrix
		inRange(MATS[2],Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),MATS[1]);
		//perform morphological operations on MATS[1]ed image to eliminate noise
		//and emphasize the filtered object(s)
		morphOps(MATS[1]);
		//pass in MATS[1]ed frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		trackFilteredObject(&ball, MATS[1], MATS[0]);

		//show frames
		imshow(windowName,  MATS[0]);
		imshow(windowName2, MATS[1]);
		//imshow(windowName1, MATS[2]);

		if(ball.detected){
			// PID COMPUTE
			config.servoX = (uint16_t)PIDCompute(&XPID, ball.x[0]);
			encodeConfig(&config, buf);
			printEncodedPack(buf);

			//bytes_written = write(fd,(void*)buf, sizeof(buf));
			printf("\n	X pulse: %d    |    Y pulse: %d\n", config.servoX, 0);
			printf("\ncount:%d | #%d Bytes written to /dev/ttyACM \n ____________________________________________\n",
				global_clock, bytes_written);

		}
		//printBall(b);
		global_clock++;
		if(waitKey(FPS) >= 0) break;

	}//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


//__EXIT ROUTINE __________________________
	printf("_______ EXIT ROUTINE _________ \n\n");

	//destroy all windows
	printf("# Destroy all windows... ");
	destroyAllWindows();
	printf("Done.\n");

	//release VideoCapture
	printf("# Release cv::VideoCapture... ");
	capture.release();
	printf("Done.\n");

	//release Mat
	printf("# Release cv::Mat... ");
	MATS[0].release();
	MATS[1].release();
	printf("Done.\n");

	//free fd and structures
	printf("# Close file descriptor and free data structures... ");
	close(fd);

	printf("Done.\n____________________________\n\n");

	return 0;
}
