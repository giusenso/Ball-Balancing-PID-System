// Testing unit

#include <sstream>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <time.h>

#include "serial/serial.h"
#include "ball_tracker/ball_tracker.h"
#include "pid/pid.h"

using namespace cv;

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//********** M A I N **********************************************************************************
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


int main(int argc, char* argv[]){

	Mat MATS[3] = {	cv::Mat(FRAME_HEIGHT,FRAME_WIDTH, CV_8UC3) , 
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					};	// Mat Array = [ webcam | masked | HSV ]
	
	cv::Mat TOOL(FRAME_HEIGHT-CONTROL_AREA, CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	cv::Mat GUI;

	//initialize camera__________________________
	VideoCapture capture;

	int CAM_NUMBER = 0;
	for ( ; CAM_NUMBER<3 ; CAM_NUMBER++){
		capture.open(CAM_NUMBER);
		if (capture.isOpened()){
			printf("# /dev/video%d successfully opened\n", CAM_NUMBER);
			break;
		}
	}
	if (CAM_NUMBER == 3){
		perror("ERROR: NO dev/video* DEVICE CONNECTED\n");
		exit(EXIT_FAILURE);
	}

	//initialize serial communication______________
	int fd = -1;
	int device_opened = openSerialCommunication(&fd);
	if(device_opened >= 0){
		setSerialAttributes(fd);
		printf("# %s successfully opened\n", serialPorts[device_opened]);
	}
	else{
		perror("\nERROR: no serial device avaible!\n");
		exit(EXIT_FAILURE);
	}

	uint8_t buf[5] = { 0, 0, 0, 0, '\n'};
	int bytes_written = 0;
	printf("# write_buffer allocated\n");

	//create ball instance
	printf("\n# create Ball object... ");
	Ball ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
	printf("Done.\n");
	printBall(ball);
	
//_ X PID SETUP __________________________________
	printf("\n# create PID objects... ");
    PID_t XPID = createPID(12, 12, 3, FRAME_WIDTH/2, true, X_MIN_ANGLE, X_MAX_ANGLE);
	PID_t YPID = createPID(12, 12, 3, FRAME_HEIGHT/2, false, Y_MIN_ANGLE, Y_MAX_ANGLE);
	printf("Done. \n");
    printPID(XPID);
	printPID(YPID);

//________________________________________________
	//Initialize data structure
	ServoConfig_t config = { 
		.servoX = X_HALF_ANGLE,
		.servoY = Y_HALF_ANGLE
	};
	printServoConfig(config);
	
//________________________________________________
	cv::Rect controlROI(SETPOINT_X-CONTROL_AREA/2,SETPOINT_Y-CONTROL_AREA/2, 
						CONTROL_AREA, CONTROL_AREA);

	char tmp;
	printf("\n # All parameters setted, ready to go...\n\n -> Press enter to start <-\n");
	scanf("%c", &tmp);
	
	//Handshake routine
	printf("\n# Handshake routine... ");
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	printf("Done. \n");

	clock_t start, end;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (strcmp(argv[1],"-auto") == 0){//:::::::::::: MAIN LOOP :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		
		printf("\n# create tracksbars... ");
		createTrackbars();
		createGainTrackbars(&XPID, &YPID);
		printf("Done. \n");
	
		capture.read(MATS[0]);
		cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
		inRange(MATS[2], Scalar(63, 114, 79), Scalar(96, 255, 256), MATS[1]);
		morphOps(MATS[1]);
		trackFilteredObject(&ball, MATS[1], MATS[0]);
		
		while(true){
			start = clock();

			capture.read(MATS[0]); 									//store image to matrix
			cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);	//convert frame from BGR to HSV colorspace
			inRange(MATS[2], Scalar(63, 114, 79), Scalar(96, 255, 256), MATS[1]);
			//inRange(MATS[2], Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), MATS[1]); //filter HSV image between values and store filtered image to MATS[1]
			morphOps(MATS[1]); 								//eliminate noise and emphasize the filtered object(s)
			trackFilteredObject(&ball, MATS[1], MATS[0]); 	//this function return the x and y ball coordinates
			drawObjectV2(ball, MATS[0], false);

			cvtColor(MATS[1], MATS[1], COLOR_GRAY2BGR);
			cv::vconcat(TOOL, MATS[1], MATS[1]);
			cv::hconcat(MATS[1], MATS[0], GUI); 		// Accept array + size
			imshow(windowName, GUI); 				//show camera feed	


			if(ball.detected){
				PIDCompute(&XPID, &YPID, ball);
				config.servoX = XPID.output[0];
				config.servoY = YPID.output[0];

				encodeConfig(&config, buf);	//Create Packet
				bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
				if(bytes_written != 5){
					perror("Error: write() syscall failed");
					exit(EXIT_FAILURE);
				}
			}

			if(waitKey(10) >= 0) break;
			
			end = clock();
			XPID.dt = YPID.dt = (float)(end - start)/CLOCKS_PER_SEC;
		}//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (strcmp(argv[1],"-fast") == 0){//:::::::::::: MAIN LOOP :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		
		printf("\n# create tracksbars... ");
		//createTrackbars();
		createGainTrackbars(&XPID, &YPID);
		printf("Done. \n");

		capture.read(MATS[0]);
		cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
		inRange(MATS[2], Scalar(63, 114, 79), Scalar(96, 255, 256), MATS[1]);
		morphOps(MATS[1]);
		trackFilteredObject(&ball, MATS[1], MATS[0]);
		
		while(true){
			start = clock();

			capture.read(MATS[0]); 									//store image to matrix
			cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);	//convert frame from BGR to HSV colorspace
			inRange(MATS[2], Scalar(63, 114, 79), Scalar(96, 255, 256), MATS[1]);
			//inRange(MATS[2], Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), MATS[1]); //filter HSV image between values and store filtered image to MATS[1]
			morphOps(MATS[1]); 								//eliminate noise and emphasize the filtered object(s)
			trackFilteredObject(&ball, MATS[1], MATS[0]); 	//this function return the x and y ball coordinates

			if(ball.detected){
				PIDCompute(&XPID, &YPID, ball);
				config.servoX = XPID.output[0];
				config.servoY = YPID.output[0];

				encodeConfig(&config, buf);	//Create Packet
				bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
				if(bytes_written != 5){
					perror("Error: write() syscall failed");
					exit(EXIT_FAILURE);
				}
			}

			if(waitKey(5) >= 0) break;
			
			end = clock();
			XPID.dt = YPID.dt = (float)(end - start)/CLOCKS_PER_SEC;
		}//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (strcmp(argv[1],"-manual") == 0){
		config.servoX = X_HALF_ANGLE;
		config.servoY = Y_HALF_ANGLE;
		while(true){
			printf("\nEnter X Angle: ");
			scanf("%u", (unsigned int*)&config.servoX);
			printf("Enter Y Angle: ");
			scanf("%u", (unsigned int*)&config.servoY);

			encodeConfig(&config, buf);	//Create Packet
			bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
			if(bytes_written != 5){
					perror("Error: write() syscall failed");
					exit(EXIT_FAILURE);
			}
		}
	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//__EXIT ROUTINE __________________________
	printf("\n========== EXIT PROTOCOL ========== \n\n");

	//destroy all windows
	printf("# Destroy all windows... ");
	cv::destroyAllWindows();
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

	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &config);
	printf("Done. \n");
	
	printf("\n===================================\n\n");

	exit(EXIT_SUCCESS);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
