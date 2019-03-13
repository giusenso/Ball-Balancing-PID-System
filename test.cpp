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
	uint8_t buf[5] = { 0, 0, 0, 0, '\n'};
	int bytes_written = 0;
	printf("# write_buffer allocated\n");

	//create ball instance
	Ball ball = createBall(0, 0);
	printBall(ball);
	
//_ X PID SETUP __________________________________
	printf("\n# creating PIDs... ");
    PID_t XPID = createPID(25, 0, 0, FRAME_WIDTH/2);
	PID_t YPID = createPID(-25, 0, 0, FRAME_HEIGHT/2);
	printf("Done. \n");
    printPID(XPID);
	printPID(YPID);

//________________________________________________
	//Initialize data structure
	ServoConfig_t config = { 
		.servoX = HALF_ANGLE,
		.servoY = HALF_ANGLE
	};
	printServoConfig(config);
	
	//Setup servos
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	printf("\n# %d Bytes written to /dev/ttyACM \n", bytes_written);
//________________________________________________

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

	clock_t start, end;
	float average_dt = 0.0, time_from_beginning = 0.0;
	int frame_counter = 0;

	char temp;
	printf("\n### All parameters setted, ready to go...\nPress enter to start\n");
	scanf("%c", &temp);

	//create slider bars for HSV filtering
	createTrackbars();

	//:::::::::::::: MAIN LOOP ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	while(true){
		start = clock();

		//store image to matrix
		capture.read(MATS[0]);

		//convert frame from BGR to HSV colorspace
		cvtColor(MATS[0],MATS[2],COLOR_BGR2HSV);

		//filter HSV image between values and store filtered image to MATS[1]
		//inRange(MATS[2], Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), MATS[1]);
		inRange(MATS[2], Scalar(59, 88, 108), Scalar(85, 255, 254), MATS[1]);
		
		//eliminate noise and emphasize the filtered object(s)
		morphOps(MATS[1]);

		//this function will return the x and y ball coordinates
		trackFilteredObject(&ball, MATS[1], MATS[0]);

		//show frames
		imshow(windowName,  MATS[0]);	// Camera feed
		imshow(windowName1, MATS[1]);	// Threshold
		//imshow(windowName2, MATS[2]);	// HSV

		//printBall(ball);
		if(ball.detected){
			//PID compute
			//config.servoX = (uint16_t)PIDCompute(&XPID, ball.x[0]);
			config.servoY = (uint16_t)PIDCompute(&YPID, ball.y[0]);
			//printPID(XPID);
			printPID(YPID);
			//printServoConfig(config);

			//Create Packet
			encodeConfig(&config, buf);
			//printEncodedPack(buf);

			//Send packet
			bytes_written = write(fd,(void*)buf, sizeof(buf));
			//printf("\n# %d Bytes written to /dev/ttyACM \n", bytes_written);
		}

		if(waitKey(20) >= 0) break;
		
		frame_counter++;
		time_from_beginning += XPID.dt;
		
		end = clock();
		XPID.dt = YPID.dt = (float)(end - start) / CLOCKS_PER_SEC;
	}//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	average_dt = (time_from_beginning/frame_counter);
	printf("\naverage_time: %.4lf\n", average_dt);


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
	MATS[2].release();
	printf("Done.\n");

	//free fd and structures
	printf("# Close file descriptor and free data structures... ");
	close(fd);

	printf("Done.\n____________________________\n\n");

	return 0;
}
