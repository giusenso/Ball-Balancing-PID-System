
#include "modes.h"
#include "../utils.h"


int standard_mode(){

const char* pid_data_file_name = "settings/pid_data.txt";
const char* hsv_data_file_name = "settings/hsv_data.txt";

int fd = -1;
int ret __attribute__((unused)); /*for unused variables suppression*/
 
//===== SETUP OPENCV DATA STRUCTURES ==========================================
	Mat MATS[3] = {	cv::Mat(FRAME_HEIGHT,FRAME_WIDTH, CV_8UC3) , 
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
				  };	// Mat Array = [ webcam | masked | HSV ]
	
	cv::Mat GUI(FRAME_HEIGHT, FRAME_WIDTH+CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	cv::Mat TOOL(FRAME_HEIGHT-CONTROL_AREA, CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
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
		memset(buf, 0, sizeof(buf));
		printf("# write_buffer allocated\n");


//===== SETUP DATA STRUCTURES =================================================
	
	//_ Ball setup __________________________________
		printf("\n# create Ball object... ");
		Ball_t ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
		printf("Done.\n");
		printBall(ball);
		
		//_ PID setup ___________________________________
		printf("\n# create PID objects... ");
		float x_gains[3], y_gains[3];
		arrayFromTextFile(pid_data_file_name, x_gains, 1);
		arrayFromTextFile(pid_data_file_name, y_gains, 2);

		PID_t XPID = createPID(	x_gains[0], x_gains[1], x_gains[2], 
								FRAME_WIDTH/2, true, X_MIN_ANGLE, X_MAX_ANGLE);
		PID_t YPID = createPID(	y_gains[0], y_gains[1], y_gains[2], 
								FRAME_HEIGHT/2, false, Y_MIN_ANGLE, Y_MAX_ANGLE);
								
		printf("Done. \n");
		printPID(XPID);
		printPID(YPID);

	//_ Initialize servo config___________________________
		ServoConfig_t config = { 
			.servoX = X_HALF_ANGLE,
			.servoY = Y_HALF_ANGLE
		};
		printServoConfig(config);
	

		char tmp;
		printf("\n # All parameters setted, ready to go...\n\
				\n -> Press enter to start <- \n");
		ret = scanf("%c", &tmp);
		
		/* Handshake with avr______________________________
		*  [not a real handshake, this is needed to setup the avr)]
		*/
		printf("# Handshake... ");
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
//===== TEST RUN ==============================================================
		float min_hsv[3], max_hsv[3];
		arrayFromTextFile(hsv_data_file_name, min_hsv, 1);
		arrayFromTextFile(hsv_data_file_name, max_hsv, 2);
		H_MIN = (int)min_hsv[0];
		S_MIN = (int)min_hsv[1];
		V_MIN = (int)min_hsv[2];
		H_MAX = (int)max_hsv[0];
		S_MAX = (int)max_hsv[1];
		V_MAX = (int)max_hsv[2];
		
		capture.read(MATS[0]);
		cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
		inRange (	MATS[2],
					Scalar(H_MIN, S_MIN, V_MIN),
					Scalar(H_MAX, S_MAX, V_MAX), 
					MATS[1]
				);
		morphOps(MATS[1]);
		trackFilteredObject(&ball, MATS[1]);
		cvtColor(MATS[1], MATS[1], COLOR_GRAY2BGR);
		cv::vconcat(MATS[1], TOOL, MATS[1]);
		cv::hconcat(MATS[1], MATS[0], GUI);

	//_ center window based on screen resolution (at least 1280p)
		cv::Point gui_pos = cv::Point(1,1);
		if( getWindowPos(&gui_pos, GUI) != 0 ){
			exit(EXIT_FAILURE);
		}
		cv::imshow(windowName, GUI);
		cv::moveWindow(windowName, gui_pos.x, gui_pos.y-100);
		total_start = clock();

//=============================================================================
//:::::::::::::: MAIN LOOP ::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
		
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
				//compute new servo pulses
				PIDCompute(&XPID, &YPID, ball);
				config.servoX = XPID.output[0];
				config.servoY = YPID.output[0];

				//encode and send to avr
				encodeConfig(&config, buf);	//Create Packet
				bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
				if(bytes_written != 5){
					perror("Error: write() syscall failed");
					exit(EXIT_FAILURE);
				}
			}
			printf("output:  X = %d , Y = %d , dt = %.4lf\n",XPID.output[0], YPID.output[0], YPID.dt*9.1);
			end = clock();
			
			//update dt based on frame rate
			//XPID.dt = YPID.dt = (float)(end - start)/CLOCKS_PER_SEC;

			frame_counter++;
			if(waitKey(1) >= 0) break;
		}
		
	//_ print frame rate info
		total_end = clock();
		tot = 7*((double)(total_end-total_start)/CLOCKS_PER_SEC);
		printf("\nTime: %d frames\n", frame_counter);
		printf("Time: %.3lf seconds\n", tot);
		printf("*******************\n AVERAGE FPS: %lf \n********************", 
			(float)frame_counter/tot);
	
//=============================================================================
//::::::: EXIT ROUTINE ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================

//__EXIT ROUTINE __________________________
	printf("\n========== EXIT PROTOCOL ========== \n\n");

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
	TOOL.release();
	GUI.release();
	printf("Done.\n");

	//close serial
	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &config);
	printf("Done. \n");
	
	printf("\n===================================\n\n");

	return 0;
}