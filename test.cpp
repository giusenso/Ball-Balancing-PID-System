// Testing unit

#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

#include "serial_port/serial_port.h"
#include "ball_tracker/ball_tracker.h"
#include "pid/pid.h"

using namespace cv;


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

	//enable this to capture mp4 file
	capture.open("/home/jius/Desktop/ball-tracking-platform/ball_tracker/samples/test2.mp4");
	
	/*
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
	*/

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	

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

	//initialize write_buffer_______________________
	uint8_t* write_buffer = (uint8_t*)malloc(sizeof(ServoConfig_t));	//devo allocare di piu'?
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


	//set mask for ball detection
	H_MIN = 41;
	H_MAX = 69;
	S_MIN = 66;
	S_MAX = 255;
	V_MIN = 48;
	V_MAX = 185;

	usleep(1000000); //for debug
	int print_counter = 0; //for debug
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
		printf("# [%d] %s written to ttyACM0 \n", print_counter, write_buffer);
		//printServoConfig(config);
		printf("# [%d] %d Bytes written to ttyACM0\n\n", print_counter, bytes_written);
		print_counter++;	//for debug

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

	}//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


//__EXIT ROUTINE __________________________
	printf("_____ EXIT ROUTINE _________ \n\n");

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
	cameraFeed.release();
	threshold.release();
	printf("Done.\n");

	//free fd and structures
	printf("# Close file descriptor and free data structures... ");
	close(fd);
	free(config);
	free(write_buffer);

	printf("Done.\n____________________________\n\n");

	return 0;
}