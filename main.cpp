// Testing unit

#include <sstream>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <time.h>

#include "serial/serial.h"
#include "ball_tracker/ball_tracker.h"
#include "pid/pid.h"

using namespace cv;

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//********** M A I N **********************************************************************************
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int main(int argc, char* argv[]){

	Mat MATS[3];	// Mat Array = [ webcam | masked | HSV ]

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

	uint8_t buf[5] = { 0, 0, 0, 0, '\n'};
	int bytes_written = 0;
	printf("# write_buffer allocated\n");

	//create ball instance
	printf("\n# create Ball object... ");
	Ball ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
	printf(" Done.\n");
	printBall(ball);
	
//_ X PID SETUP __________________________________
	printf("\n# create PID objects... ");
    PID_t XPID = createPID(15, 8, 3, FRAME_WIDTH/2, true, X_MIN_ANGLE, X_MAX_ANGLE);
	PID_t YPID = createPID(18, 9, 3, FRAME_HEIGHT/2, false, Y_MIN_ANGLE, Y_MAX_ANGLE);
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
	sleep(100000);
	*/

	char temp;
	printf("\n # All parameters setted, ready to go...\n\n -> Press enter to start <-\n");
	scanf("%c", &temp);

	printf("\n# create tracksbars... ");
	createTrackbars();
	createGainTrackbars(&XPID, &YPID);
	printf("Done. \n");
	
	//Handshake routine
	printf("\n# Handshake routine... ");
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	printf("Done. \n");

	clock_t start, end;

	if (strcmp(argv[1],"-manual") == 0){
		config.servoX = X_HALF_ANGLE;
		config.servoY = Y_HALF_ANGLE;
		while(true){
			printf("\nEnter X Angle: ");
			scanf("%u", &config.servoX);
			printf("Enter Y Angle: ");
			scanf("%u", &config.servoY);
			encodeConfig(&config, buf);	//Create Packet
			bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
		}
	}
	
	if (strcmp(argv[1],"-auto") == 0){//:::::::::::: MAIN LOOP :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		cv::Rect controlROI(SETPOINT_X-CONTROL_AREA/2,SETPOINT_Y-CONTROL_AREA/2, 
							CONTROL_AREA, CONTROL_AREA);
		int _l = 100;
		cv::Rect ballROI(ball.x[0]-_l, ball.y[0]-_l, _l+_l, _l+_l);

		while(true){
			start = clock();

			capture.read(MATS[0]); //store image to matrix

			cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV); //convert frame from BGR to HSV colorspace

			inRange(MATS[2], Scalar(63, 114, 79), Scalar(96, 255, 256), MATS[1]);
			//filter HSV image between values and store filtered image to MATS[1]
			//inRange(MATS[2], Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), MATS[1]);
			
			/*
			if(ball.detected){
				if(ball.x[0] > _l) ballROI.x = ball.x[0] - _l;
				else ballROI.x = 0;
				if(ball.y[0] > _l) ballROI.y = ball.y[0] - _l;
				else ballROI.y = 0;
				MATS[1] = MATS[1](ballROI);
			}
			*/

			morphOps(MATS[1]); //eliminate noise and emphasize the filtered object(s)
			trackFilteredObject(&ball, MATS[1], MATS[0]); //this function will return the x and y ball coordinates

			imshow(windowName,  MATS[0]); 	//show camera feed
			imshow(windowName1, MATS[1]);	// Threshold

			if(ball.detected){

				PIDCompute(&XPID, &YPID, ball);
				config.servoX = XPID.output[0];
				config.servoY = YPID.output[0];

				encodeConfig(&config, buf);	//Create Packet

				bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
				//printf("\n# %d Bytes written to /dev/ttyACM \n", bytes_written);
			}

			if(waitKey(5) >= 0) break;
			
			end = clock();
			XPID.dt = YPID.dt = (float)(end - start)/CLOCKS_PER_SEC;
		}//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	}

//__EXIT ROUTINE __________________________
	printf("\n========== EXIT PROTOCOL ========== \n\n");

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

	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &config);
	printf("Done. \n");
	
	printf("\n===================================\n\n");

	return 0;
}
