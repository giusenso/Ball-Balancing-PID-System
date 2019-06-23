/**
 * @file standard_mode.cpp
 * @author Giuseppe Sensolini
 * @brief STANDARD MODE
 * @version 2.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "modes.h"
#include "../utils.h"

//=============================================================================
//:::::::::::::: STANDARD :::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
int standard_mode(){

	const char* pid_data_file_name = "settings/pid_data.txt";
	const char* hsv_data_file_name = "settings/hsv_data.txt";

	int fd = -1;
	int ret __attribute__((unused)); /*for unused variables suppression*/

//===== SETUP OPENCV DATA STRUCTURES ==========================================

	// MATS = [ cameraFeed | threshold | HSV ]
	Mat MATS[3] = {	cv::Mat(FRAME_HEIGHT,FRAME_WIDTH, CV_8UC3) ,
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
				  };

	// 400x400px rect
	cv::Rect controlROI(SETPOINT_X-CONTROL_AREA/2,SETPOINT_Y-CONTROL_AREA/2,
						CONTROL_AREA, CONTROL_AREA);

//_ Open camera stream ____________________________
	cv::VideoCapture capture;
	int CAM_NUMBER = 0;
	for ( ; CAM_NUMBER<3 ; CAM_NUMBER++){
		capture.open(CAM_NUMBER);
		if ( capture.isOpened() ){
			printf("# /dev/video%d successfully opened\n", CAM_NUMBER);
			break;
		}
	}
	if ( CAM_NUMBER == 3 ){
		perror("ERROR: NO dev/video* DEVICE CONNECTED");
		exit(EXIT_FAILURE);
	}

//===== INITIALIZE SERIAL COMMUNICATION =======================================

	int device_opened = openSerialCommunication(&fd);
	if( device_opened >= 0 ){
		setSerialAttributes(fd);
		printf("# %s successfully opened\n", serialPorts[device_opened]);
	}
	else{
		perror("\nERROR: NO /dev/ttyACM* DEVICE CONNECTED");
		exit(EXIT_FAILURE);
	}

	int bytes_written = 0;
	uint8_t buf[5];
	printf("# write_buffer allocated\n");

//===== SETUP DATA STRUCTURES =================================================
/*
//_ PID setup ___________________________________
	printf("\n# create PID objects... ");
	float x_gains[3], y_gains[3];
	arrayFromTextFile(pid_data_file_name, x_gains, 1);
	arrayFromTextFile(pid_data_file_name, y_gains, 2);
*/
//_ Ball setup ___________________________________
	bool ball_detected = false;
	Point_t ball_pos = {
		.x = FRAME_WIDTH/2,
		.y = FRAME_HEIGHT/2
	};
	
//_ Read HSV mask ____________________________________
	float min_hsv[3], max_hsv[3];
	arrayFromTextFile(hsv_data_file_name, min_hsv, 1);
	arrayFromTextFile(hsv_data_file_name, max_hsv, 2);
	H_MIN = (int)min_hsv[0];
	S_MIN = (int)min_hsv[1];
	V_MIN = (int)min_hsv[2];
	H_MAX = (int)max_hsv[0];
	S_MAX = (int)max_hsv[1];
	V_MAX = (int)max_hsv[2];

//_ stats variables____________________________________
	clock_t total_start, total_end;
	int frame_counter = 0;
	int packet_counter = 0;
	double tot;
	
//_ center window based on screen resolution (at least 1280p)
	cv::Point window_pos = cv::Point(1,1);
	if( getWindowPos(&window_pos, MATS[1]) != 0 ){
		exit(EXIT_FAILURE);
	}
	//else cv::moveWindow(windowName, window_pos.x, window_pos.y-100);
//=============================================================================

	char tmp;
	printf("\n # All parameters setted, ready to go...\n\
			\n -> Press enter to start <- \n");
	ret = scanf("%c", &tmp);

//_ Handshake with avr________________________________
	/*  [not a real handshake, this is needed to setup the avr)] */
	printf("# Handshake... ");
	memcpy(buf, &ball_pos, 4);
	buf[4] = '\n';
	bytes_written = write(fd, (void*)buf, sizeof(buf));
	bytes_written = write(fd, (void*)buf, sizeof(buf));
	printf("Done. \n");
	
//=============================================================================
//:::::::::::::: MAIN LOOP ::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================

	while(true){

		capture.read(MATS[0]);	//store image to matrix
		cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
		inRange(	MATS[2],
					Scalar(H_MIN, S_MIN, V_MIN),
					Scalar(H_MAX, S_MAX, V_MAX),
					MATS[1]
				);

		morphOps(MATS[1]);
		trackFilteredObject(&ball_detected, &ball_pos, MATS[1]);
		cv::imshow(windowName,MATS[1]);

		if(ball_detected){
			memcpy(buf, &ball_pos, 4);
			bytes_written = write(fd, (void*)buf, sizeof(buf)); //Send packet
			if(bytes_written != 5){
				perror("Error: write() syscall failed");
				exit(EXIT_FAILURE);
			}
			printf(" xb: %d    yb: %d\n", ball_pos.x, ball_pos.y);
			packet_counter++;
		}

		frame_counter++;
		if(waitKey(1) >= 0) break;
	}

//=============================================================================
//::::::: EXIT ROUTINE ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
	
//_ print frame rate info_______________________
	total_end = clock();
	tot = 7*((double)(total_end-total_start)/CLOCKS_PER_SEC);
	printf("\n*******************\
			\nTime: %.2lf sec\nFrames: %d\nPackets: %d\
			\nFPS: %.2lf \n********************",
			 tot, frame_counter, packet_counter, (float)frame_counter/tot);

//_ CLOSE EVERYTHING____________________________
	printf("\n========== EXIT ROUTINE ========== \n\n");

	//destroy all windows
	printf("# Destroy all windows... ");
	cv::destroyWindow(windowName);
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

	//close serial
	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &ball_pos);
	printf("Done. \n");

	printf("\n===================================\n\n");

	return 0;
}
