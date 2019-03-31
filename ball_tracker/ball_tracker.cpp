/**
 * @file ball_tracker.cpp
 * @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
 * 
 * @brief 
 * 		- open camara and get video stream
 * 		- hsv mask tracksbars
 * 		- detect object with HSV mask
 * 		- Morphological trasformations
 * 		- draw data to windows
 * 		- circle detection
 * 		- get screen size and compute window position
 * 
 * @version 1.2
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#include "ball_tracker.h"

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

const String windowName = "Camera Feed";
const String trackbarWindowName = "HSV Trackbars";
const String gainTrackbarWindowName = "PID GAINS";

/**
 * @brief Get the window position based on screen size
 * 
 * @param point 
 * @param mat 
 * @return -1 in case of error, else 0
 */
int getWindowPos(cv::Point* point, cv::Mat mat){
	const char* command = "xrandr | grep '*'";
 	FILE* fpipe = (FILE*)popen(command,"r");
 	char line[256];
	memset(line, 0, sizeof(line));
 	fgets(line, sizeof(line), fpipe);

	int x_res = 0, y_res = 0, i = 0, start = 0, end = 0;

	while(true){	//find x res
		if(line[i] == ' '){
			start = i+1;
		}
		if(line[i] == 'x'){
			end = i;
			char x_buf[end-start];
			memset(x_buf, 0, sizeof(x_buf));
			memcpy(x_buf, line+start, end-start);
			x_res = atoi(x_buf);
			break;
		}
		i++;
	}
	start = end+1;
	while(true){	//find y res
		if(line[i] == ' '){
			end = i;
			char y_buf[end-start];
			memset(y_buf, 0, sizeof(y_buf));
			memcpy(y_buf, line+start, end-start);
			y_res = atoi(y_buf);
			break;
		}
		i++;
	}
 	pclose(fpipe);

	if(x_res<360||x_res>4096||y_res<240||y_res>4096){
		fprintf(stderr, "Screen resolution wrong!\n");
		return -1;
	}
	
	if(x_res<720) x_res = 720;
	if(y_res<480) x_res = 480;

	point->x = (x_res - mat.cols) / 2;
	point->y = (y_res - mat.rows) / 2;

	return 0;
}

/**
 * @brief called whenever a trackbar position is changed
 */
void on_trackbar( int, void* ){
	//empty function
}

/**
 * @brief convert integer to string
 * 
 * @param number 
 * @return String 
 */
String intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

/**
 * @brief Create a Trackbars object for HSV mask
 * 
 */
void createTrackbars(){

	//create window for trackbars
    namedWindow(trackbarWindowName, 0);

	//create memory to store trackbar name on window
	char TrackbarName[16];
	sprintf( TrackbarName, "H_MIN");
	sprintf( TrackbarName, "H_MAX");
	sprintf( TrackbarName, "S_MIN");
	sprintf( TrackbarName, "S_MAX");
	sprintf( TrackbarName, "V_MIN");
	sprintf( TrackbarName, "V_MAX");

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

/**
 * @brief Create a Gain Trackbars object for PID gainss
 * 		!!!unused function!!!
 * @param XPID 
 * @param YPID 
 */
void createGainTrackbars(PID_t* XPID, PID_t* YPID){

    namedWindow(gainTrackbarWindowName, 0);

	char TrackbarName[16];
	sprintf( TrackbarName, "X_Kp");
	sprintf( TrackbarName, "X_Ki");
	sprintf( TrackbarName, "X_Kd");
	sprintf( TrackbarName, "Y_Kp");
	sprintf( TrackbarName, "Y_Ki");
	sprintf( TrackbarName, "Y_Kd");

	createTrackbar( "X_Kp", gainTrackbarWindowName, (int*)&(XPID->Kp), P_MAX, on_trackbar );
    createTrackbar( "X_Ki", gainTrackbarWindowName, (int*)&(XPID->Ki), I_MAX, on_trackbar );
    createTrackbar( "X_Kd", gainTrackbarWindowName, (int*)&(XPID->Kd), D_MAX, on_trackbar );
	createTrackbar( "Y_Kp", gainTrackbarWindowName, (int*)&(YPID->Kp), P_MAX, on_trackbar );
    createTrackbar( "Y_Ki", gainTrackbarWindowName, (int*)&(YPID->Ki), I_MAX, on_trackbar );
    createTrackbar( "Y_Kd", gainTrackbarWindowName, (int*)&(YPID->Kd), D_MAX, on_trackbar );
}


/**
 * @brief DRAW VERSION V2
 * 		second version of draw
 * 
 * @param ball Ball_t object
 * @param frame where to draw
 * @param noise_error set alway false
 */
void drawObjectV2(Ball_t ball, Mat &frame, bool noise_error){

	if(noise_error){	//////////////////////////////////////
		rectangle(	frame,
					Point(SETPOINT_X-202, SETPOINT_Y-202),
					Point(SETPOINT_X+202, SETPOINT_Y+202),
					RED, 3, LINE_8, 0);
		putText(frame, "TOO MUCH NOISE! ADJUST FILTERS", Point(180,220), 1, 1, RED , 2);
		return;
	}

	if (ball.detected){	//////////////////////////////////////
		//draw square area
		rectangle(	frame,
					Point(SETPOINT_X-CONTROL_AREA/2-1, SETPOINT_Y-CONTROL_AREA/2-1),
					Point(SETPOINT_X+CONTROL_AREA/2+1, SETPOINT_Y+CONTROL_AREA/2+1),
					CYAN, 3, LINE_8, 0);
		putText(frame, "BALL FOUND", Point(123,32), 1, 1, GREEN, 2);

		//draw setpoint area
		circle(frame, Point(SETPOINT_X,SETPOINT_Y), 2, CYAN, 3);
		rectangle(	frame,
					Point(SETPOINT_X-30, SETPOINT_Y-30),
					Point(SETPOINT_X+30, SETPOINT_Y+30),
					ORANGE, 1, LINE_4, 0);


		//draw ball lines for position spot
		rectangle(	frame,
					Point(ball.x[0]-28, ball.y[0]-28),
					Point(ball.x[0]+28, ball.y[0]+28),
					GREEN, 1, LINE_8, 0);

		//draw velocity arrow
		arrowedLine(frame, 	Point(ball.x[0] , ball.y[0]),
							Point(ball.x[0]+ball.smooth_dx, ball.y[0]+ball.smooth_dy),
							RED, 2, 8, 0 , 0.4);

		//draw previous positions
		for (int i=1 ; i<8 ; i++){
			circle( frame, Point(ball.x[i], ball.y[i]), 2, ORANGE, -1, 8, 0 );

		
		plotPos(ball, frame, 522, FRAME_HEIGHT/2);

		}
		//display ball info
		putText(frame,intToString(ball.y[0]),Point(ball.x[0]+2,ball.y[0]-42),1,1,BLUE,2);
		line(frame, Point(ball.x[0], 0), Point(ball.x[0], FRAME_HEIGHT), BLUE, 1);
		line(frame, Point(0, ball.y[0]), Point(FRAME_WIDTH, ball.y[0]), BLUE, 1);
		putText(frame,intToString(ball.y[0]),Point(ball.x[0]+2,ball.y[0]-42),1,1,BLUE,2);
		putText(frame,intToString(ball.x[0]),Point(ball.x[0]+40,ball.y[0]+14),1,1,BLUE,2);

		return;
	}

	//////////////////////////////////////
	//draw square area
	rectangle(	frame,
		Point(SETPOINT_X-202, SETPOINT_Y-202),
		Point(SETPOINT_X+202, SETPOINT_Y+202),
		ORANGE, 3, LINE_8, 0);

	putText(frame, "BALL NOT FOUND", Point(123,32), 1, 1, RED , 2);
	return;

}

/**
 * @brief Draw a real time chart
 * [called in drawObjectV2]
 * 
 * @param b Ball_t object
 * @param frame where to draw
 * @param x
 * @param y 
 */
inline void plotPos(Ball_t b, Mat &frame, uint16_t x, uint16_t y){
	putText(frame, "X", Point(x+5, y-120), 1, 1, DARK_GREEN , 2);
	putText(frame, "Y", Point(x+5, y+80), 1, 1, DARK_GREEN , 2);
	line(frame, Point(x, y-100), Point(x+200, y-100), CYAN, 2);
	line(frame, Point(x, y+100), Point(x+200, y+100), CYAN, 2);

	for(int i=1 ; i<8 ; i++){
		line(frame, Point(x+i*10, (y-100+(b.x[i-1]-SETPOINT_X)/3)),
			Point(x+i*20, (y-100+(b.x[i]-SETPOINT_X)/3)), DARK_GREEN, 2);
		line(frame, Point(x+i*10, (y+100+(b.y[i-1]-SETPOINT_Y)/3)),
			Point(x+i*20, (y+100+(b.y[i]-SETPOINT_Y)/3)), DARK_GREEN, 2);
	}
}


/**
 * @brief create structuring element that will be used to "dilate" and "erode" image
 * 
 * @param thresh Threshold cv::Mat
 */
void morphOps(Mat &thresh){

	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));

    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(5,5));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);

	morphologyEx(thresh, thresh, MORPH_CLOSE, Mat::ones(5, 5, CV_8U));

	//release
	erodeElement.release();
	dilateElement.release();
}

/**
 * @brief Track Filtered Object
 * detect the ball from the treshold Mat,
 * and call updateBall()/createBall() function.
 * 
 * @param ball Ball_t object
 * @param threshold 
 */
void trackFilteredObject(Ball_t* ball, Mat threshold){
	//bool noise_error = false;

	//these two vectors needed for output of findContours
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(threshold,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	//use moments method to find our filtered object
	float refArea = 0;

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				float area = moment.m00;

                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					if(ball->detected){
						//circleDetector(cameraFeed, threshold);
						updateBall(ball,(moment.m10/area)+(FRAME_WIDTH-CONTROL_AREA)/2, 
											(moment.m01/area)+(FRAME_HEIGHT-CONTROL_AREA)/2);
						refArea = area;
					}
					else {
						*ball = createBall(	(moment.m10/area)+(FRAME_WIDTH-CONTROL_AREA)/2,
											(moment.m01/area)+(FRAME_HEIGHT-CONTROL_AREA)/2);
						ball->detected = true;
					}
				}
			}
		}
		/*
		else{
			noise_error = true;
			*ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
		}*/
	}
	else{
		ball->detected = false;
	}
}

/**
 * @brief Use HoughCircles() to find ball
 * [not actually used]
 * 
 * @param cameraFeed 
 * @param threshold 
 */
void circleDetector(Mat cameraFeed, Mat threshold){

	Mat gray;
	threshold.copyTo(gray);
	//cvtColor(threshold, gray, CV_BGR2GRAY);
	GaussianBlur( gray, gray, Size(7, 7), 1.8, 1.8 );

	std::vector<Vec3f> circles;
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 35, 100, 25, 10, 45);

	for(size_t i = 0 ; i < circles.size() ; i++){
		printf("%d    radius: %.1f \n", (int)i+1, circles[i][2]);

		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		short radius = cvRound(circles[i][2]);

		circle( cameraFeed, center, 3, ORANGE, -1, 8, 0 );     // circle center
		circle( cameraFeed, center, radius+1, GREEN, 2, 8, 0 );  // circle outline
  	}
	gray.release();
}


/**
 * @brief draw in the blank space
 * [to be implemented...]
 * 
 * @param DATA blank space
 * @param XPID 
 * @param YPID 
 */
void drawLiveData(Mat &DATA, PID_t XPID, PID_t YPID){
	putText(DATA, intToString(XPID.output[0]), Point(5,25), 1, 2, BLUE, 2);
	putText(DATA, intToString(YPID.output[0]), Point(205,25), 1, 2, BLUE, 2);
	//...
}