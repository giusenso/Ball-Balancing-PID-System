/**
 * @file main.cpp
 * @author Giuseppe Sensolini
 * 
 * @brief 		BALL BALANCING PID SYSTEM
 * @repository 	https://github.com/JiuSenso/Ball-Balancing-PID-System.git
 * 
 * 		after compiling it(see README.md for details)
 * 		can be launched with different option flags:
 * 		
 * 		[1] "./run -settings"
 * 			set pid gains and computer vision parameters
 * 
 * 		[2]	"./run -auto"
 * 			start auto mode: better performance but minimal GUI
 * 
 * 		[3]	"./run -debug"
 * 			start debug mode: a better GUI and print utilities,
 * 			little bit slower	
 * 
 * 		[4]	"./run -manual"
 * 			platform can be controlled directly from terminal.
 * 
 * 		*note: one and only one flag can be used
 *		
 * 		
 * @version 1.3
 * @date 2019-03-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <time.h>

#include "serial/serial.h"
#include "ball_tracker/ball_tracker.h"
#include "pid/pid.h"
#include "settings/file_handler.h"


const char* pid_data_file_name = "settings/pid_data.txt";
const char* hsv_data_file_name = "settings/hsv_data.txt";


int main(int argc, char* argv[]){

//===== SETUP OPENCV DATA STRUCTURES =====================================
	
	Mat MATS[3] = {	cv::Mat(FRAME_HEIGHT,FRAME_WIDTH, CV_8UC3) , 
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
				  };	// Mat Array = [ webcam | masked | HSV ]
	
	cv::Mat TOOL(FRAME_HEIGHT-CONTROL_AREA, CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	cv::Mat GUI(FRAME_HEIGHT, FRAME_WIDTH+CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	cv::Rect controlROI(SETPOINT_X-CONTROL_AREA/2,SETPOINT_Y-CONTROL_AREA/2, 
						CONTROL_AREA, CONTROL_AREA);
	
	// open camera stream ____________________________
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
		perror("ERROR: NO dev/video* DEVICE CONNECTED\n");
		exit(EXIT_FAILURE);
	}

	
	if (strcmp(argv[1],"-auto") == 0){

//===== INITIALIZE SERIAL COMMUNICATION ==================================
		int fd = -1;
		int device_opened = openSerialCommunication(&fd);
		if( device_opened >= 0 ){
			setSerialAttributes(fd);
			printf("# %s successfully opened\n", serialPorts[device_opened]);
		}
		else{
			perror("\nERROR: no serial device avaible!\n");
			exit(EXIT_FAILURE);
		}

		int bytes_written = 0;
		uint8_t buf[5];
		memset(buf, 0, sizeof(buf));
		printf("# write_buffer allocated\n");


//===== SETUP DATA STRUCTURES ===========================================
	
		//_ Ball setup __________________________________
		printf("\n# create Ball object... ");
		Ball ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
		printf("Done.\n");
		printBall(ball);
		
		//_ PID setup ___________________________________
		printf("\n# create PID objects... ");
		float x_gains[3], y_gains[3];
		arrayFromTextFile(pid_data_file_name, x_gains, 1);
		arrayFromTextFile(pid_data_file_name, y_gains, 2);
		PID_t XPID = createPID(x_gains[0], x_gains[1], x_gains[2], FRAME_WIDTH/2, true, X_MIN_ANGLE, X_MAX_ANGLE);
		PID_t YPID = createPID(y_gains[0], y_gains[1], y_gains[2], FRAME_HEIGHT/2, false, Y_MIN_ANGLE, Y_MAX_ANGLE);
		printf("Done. \n");
		printPID(XPID);
		printPID(YPID);

	//_ Initialize servo config___________________________
		ServoConfig_t config = { 
			.servoX = X_HALF_ANGLE,
			.servoY = Y_HALF_ANGLE
		};
		printServoConfig(config);
	

		char tmp = 0;
		printf("\n # All parameters setted, ready to go...\n\n -> Press enter to start <-\n");
		scanf("%c", &tmp);
		

		/* Handshake with avr______________________________
		*  [not a real handshake, this is needed to setup the avr)]
		*/
		printf("\n# Handshake... ");
		encodeConfig(&config, buf);
		bytes_written = write(fd,(void*)buf, sizeof(buf));
		encodeConfig(&config, buf);
		bytes_written = write(fd,(void*)buf, sizeof(buf));
		printf("Done. \n");

	//_ time variables____________________________________ 
		clock_t start, end;
		clock_t total_start, total_end;
		int frame_counter = 0;
		double tot;
//===== TEST RUN =======================================================
		float min_hsv[3], max_hsv[3];
		arrayFromTextFile(hsv_data_file_name, min_hsv, 1);
		arrayFromTextFile(hsv_data_file_name, max_hsv, 2);
		H_MIN = (int)min_hsv[0];
		S_MIN = (int)min_hsv[1];
		V_MIN = (int)min_hsv[2];
		H_MAX = (int)max_hsv[0];
		S_MAX = (int)max_hsv[1];
		V_MAX = (int)max_hsv[2];
		printf("%d  %d  %d \n%d %d %d\n", H_MIN, S_MIN, V_MIN, H_MAX, S_MAX, V_MAX);
		capture.read(MATS[0]);
		cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
		inRange(MATS[2], Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), MATS[1]);
		morphOps(MATS[1]);
		trackFilteredObject(&ball, MATS[1]);
		cvtColor(MATS[1], MATS[1], COLOR_GRAY2BGR);
		cv::vconcat(MATS[1], TOOL, MATS[1]);
		cv::hconcat(MATS[1], MATS[0], GUI);

	//_ center window to the screen, based on the screen resolution (at least 1280p needed)
		cv::Point gui_pos = cv::Point(1,1);
		if( getWindowPos(&gui_pos, GUI) != 0 ){
			exit(EXIT_FAILURE);
		}
		cv::imshow(windowName, GUI);
		moveWindow(windowName, gui_pos.x, gui_pos.y-100);
		total_start = clock();

//===============================================================================================
//:::::::::::::: MAIN LOOP ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//===============================================================================================
		
		while(true){
			start = clock();

			capture.read(MATS[0]);	//store image to matrix

			cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);

			inRange(	MATS[2],
						Scalar(H_MIN, S_MIN, V_MIN),
						Scalar(H_MAX, S_MAX, V_MAX),
						MATS[1]
					);
			
			morphOps(MATS[1]);

			trackFilteredObject(&ball, MATS[1]);

			cv::imshow(windowName,MATS[1]); //or use cvRoutine() for extended gui

			if(ball.detected){

			//_ compute new servo pulses
				PIDCompute(&XPID, &YPID, ball);
				config.servoX = XPID.output[0];
				config.servoY = YPID.output[0];

			//_ encode and send to avr
				encodeConfig(&config, buf);	//Create Packet
				bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
				if(bytes_written != 5){
					perror("Error: write() syscall failed");
					exit(EXIT_FAILURE);
				}
			}

			//printf("output:  X = %d , Y = %d , dt = %.4lf\n", XPID.output[0], YPID.output[0], YPID.dt*9.1);
			end = clock();

			//update dt based on frame rate
			XPID.dt = YPID.dt = (float)(end - start)/CLOCKS_PER_SEC;

			frame_counter++;
			if(waitKey(1) >= 0) break;
		}//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		
	//_ print info about frame rate
		total_end = clock();
		tot = 7*((double)(total_end-total_start)/CLOCKS_PER_SEC);
		printf("\nTime: %d frames\n", frame_counter);
		printf("Time: %.3lf seconds\n", tot);
		printf("************************\n AVERAGE FPS: %lf \n************************", 
			(float)frame_counter/tot);
	}


//===============================================================================================
//:::::::::::::: SETTINGS :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//===============================================================================================
	if (strcmp(argv[1],"-settings") == 0){
		
		FILE* file;
		char answer = 0;
		char yes[2] = {'y','Y'}, no[2] = {'n','N'};
		char to_file[256];

		while( !(answer==yes[0] || answer==yes[1] || answer==no[0] || answer==no[1]) ){
			printf("\nDo you want to set PID gains? (y/n)\n");
			scanf("%s", &answer);

			if( answer==yes[0] || answer==yes[1] ){
				float kp, ki, kd;

				printf("\nP gain: ");
				scanf(" %f", &kp);	

				printf("I gain: ");
				scanf(" %f", &ki);	

				printf("D gain: ");
				scanf(" %f", &kd);	

				printf("Save data... ");
				sprintf(to_file, "XPID[%.1f,%.1f,%.1f]\nYPID[%.1f,%.1f,%.1f]\n", kp, ki, kd, kp, ki, kd);
				stringToFile(pid_data_file_name, to_file);
				printf("Done.\n");

			}
		}

		//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		answer = 0;

		while( !(answer==yes[0] || answer==yes[1] || answer==no[0] || answer==no[1]) ){
			printf("\nDo you want to set HSV mask? (y/n)\n");
			scanf("%s", &answer);

			if( answer==yes[0] || answer==yes[1] ){			
				createTrackbars();			
				
				while(true){
					capture.read(MATS[0]);	//store image to matrix
					cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
					inRange(MATS[2], Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), MATS[1]);
					morphOps(MATS[1]);
					trackFilteredObject(&ball, MATS[1]);
					cvtColor(MATS[1], MATS[1], COLOR_GRAY2BGR);
					cv::vconcat(MATS[1], TOOL, MATS[1]);
					cv::hconcat(MATS[1], MATS[0], GUI);
					cv::imshow(windowName, GUI);
					if(waitKey(30) >= 0) break;
				}

				printf("Save data... ");
				sprintf(to_file, "MIN[%d,%d,%d]\nMAX[%d,%d,%d]\n", H_MIN, S_MIN, V_MIN, H_MAX, S_MAX, V_MAX);
				stringToFile(hsv_data_file_name, to_file);
				printf("Done.\n");
			}
		}

		printf("Exit...\n");
	}


//===============================================================================================
//::::::: EXIT ROUTINE ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//===============================================================================================

//__EXIT ROUTINE __________________________
	printf("\n========== EXIT PROTOCOL ========== \n\n");

	//destroy all windows
	printf("# Destroy all windows... ");
	cv::destroyWindow(windowName);
	if (strcmp(argv[1],"-auto") == 0){
		cv::destroyWindow(trackbarWindowName);
		cv::destroyWindow(gainTrackbarWindowName);
	}
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
	TOOL.release();
	GUI.release();
	printf("Done.\n");

	//release Trackbars
	printf("# Release cv::Mat... ");
	MATS[0].release();
	MATS[1].release();
	MATS[2].release();
	TOOL.release();
	GUI.release();
	printf("Done.\n");

	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &config);
	printf("Done. \n");
	
	printf("\n===================================\n\n");

	exit(EXIT_SUCCESS);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
