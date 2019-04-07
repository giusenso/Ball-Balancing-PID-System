/**
 * @file settings_mode.cpp
 * @author Giuseppe Sensolini
 * @brief SETTINGS MODE
 * @version 0.1
 * @date 2019-03-05
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "modes.h"
#include "../utils.h"

//=============================================================================
//:::::::::::::: SETTINGS :::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
int settings_mode(){

	const char* pid_data_file_name = "settings/pid_data.txt";
	const char* hsv_data_file_name = "settings/hsv_data.txt";

	int ret __attribute__((unused)); /*for unused variables suppression*/

	/*	PID SETTINGS	*/
	char answ = 0;
	char yes[2] = {'y','Y'}, no[2] = {'n','N'};
	char to_file[256];

	while( !(answ==yes[0] || answ==yes[1] || answ==no[0] || answ==no[1]) ){
		printf("\nDo you want to set PID gains? (y/n)\n");
		ret = scanf("%s", &answ);

		if( answ==yes[0] || answ==yes[1] ){
			float kp, ki, kd;

			printf("\nP gain: ");
			ret = scanf(" %f", &kp);

            printf("I gain: ");
			ret = scanf(" %f", &ki);

			printf("D gain: ");
			ret = scanf(" %f", &kd);
			
		//__Save on text file _________________________
			printf("Save data... ");
			sprintf(to_file, "XPID[%.1f,%.1f,%.1f]\nYPID[%.1f,%.1f,%.1f]\n",
						kp, ki, kd, kp, ki, kd);
			stringToFile(pid_data_file_name, to_file);
			printf("Done.\n");
		}
	}
	answ = 0;


	/*	HSV SETTINGS	*/
	while( !(answ==yes[0] || answ==yes[1] || answ==no[0] || answ==no[1]) ){
		printf("\nDo you want to set HSV mask? (y/n)\n");
		ret = scanf("%s", &answ);

		if( answ==yes[0] || answ==yes[1] ){

           //===== SETUP OPENCV DATA STRUCTURES ==========================================
	        Mat MATS[3] = {	cv::Mat(FRAME_HEIGHT,FRAME_WIDTH, CV_8UC3) ,
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
				  };	// Mat Array = [ webcam | masked | HSV ]

					Ball_t ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
	        cv::Mat GUI(FRAME_HEIGHT, FRAME_WIDTH+CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	        cv::Mat TOOL(FRAME_HEIGHT-CONTROL_AREA, CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	        cv::Rect controlROI(SETPOINT_X-CONTROL_AREA/2,SETPOINT_Y-CONTROL_AREA/2,
						CONTROL_AREA, CONTROL_AREA);

            //_ Open camera stream ____________________________
	        cv::VideoCapture capture;
	        int CAM_NUMBER = 1;
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

			createTrackbars();
			cv::Point gui_pos = cv::Point(1,1);
			if( getWindowPos(&gui_pos, GUI) != 0 ){
				exit(EXIT_FAILURE);

			}
				while(true){
				capture.read(MATS[0]);	//store image to matrix
				cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
				inRange(	MATS[2],
							Scalar(H_MIN, S_MIN, V_MIN),
							Scalar(H_MAX, S_MAX, V_MAX),
							MATS[1]);
				morphOps(MATS[1]);
				trackFilteredObject(&ball, MATS[1]);
				drawObjectV2(ball, MATS[0], false);

				cvtColor(MATS[1], MATS[1], COLOR_GRAY2BGR);
				cv::vconcat(MATS[1], TOOL, MATS[1]);
				cv::hconcat(MATS[1], MATS[0], GUI);
				cv::imshow(windowName, GUI);
				cv::moveWindow(windowName, gui_pos.x, gui_pos.y-100);
				if(waitKey(30) >= 0) break;
			}

		//__Save on text file _________________________
			printf("Save data... ");
			sprintf(to_file, "MIN[%d,%d,%d]\nMAX[%d,%d,%d]\n",
					H_MIN, S_MIN, V_MIN, H_MAX, S_MAX, V_MAX);
			stringToFile(hsv_data_file_name, to_file);
			printf("Done.\n");
			

		//__CLOSE EVERYTHING____________________________
			printf("\n========== EXIT PROTOCOL ========== \n\n");
			//destroy all windows
			printf("# Destroy all windows... ");
			cv::destroyWindow(windowName);
			cv::destroyWindow(windowName2);
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
			printf("\n===================================\n\n");
		}
	}
    return 0;
}
