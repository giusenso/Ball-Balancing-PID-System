#include "ball_tracker.h"

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

const String windowName = "Original Image";
const String windowName1 = "HSV Image";
const String windowName2 = "Thresholded Image";
const String windowName3 = "After Morphological Operations";
const String trackbarWindowName = "Trackbars";



//_ Function Declarations _____________________

void on_trackbar( int, void* ){
	//This function gets called whenever a
	// trackbar position is changed
}

String intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(){

	//create window for trackbars
    namedWindow(trackbarWindowName,0);

	//create memory to store trackbar name on window
	char TrackbarName[50];
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

void drawObjectV1(int x, int y, Mat &frame){

	//Draw version 1

	circle(frame,Point(x,y),24,Scalar(0,255,0),3);
	//vertical line
    if(y-25>0)
    line(frame,Point(x,0),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);

	//orizzontal line
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

void drawObjectV2(int x, int y, Mat &frame){

	//_ Draw version 2 ____________________________
	
	//draw setpoint area
	circle(frame, Point(SETPOINT_X,SETPOINT_Y), 2, DARK_GREEN, 3);
	rectangle(	frame,
				Point(SETPOINT_X-TOLLERANCE, SETPOINT_Y-TOLLERANCE),
				Point(SETPOINT_X+TOLLERANCE, SETPOINT_Y+TOLLERANCE),
				DARK_GREEN, 6, LINE_8, 0);


	//draw ball lines for position spot
	circle(frame,Point(x,y),34,GREEN, 1);

    line(frame,Point(x,0),Point(x,FRAME_HEIGHT),RED, 2);
	line(frame,Point(0,y),Point(FRAME_WIDTH,y),BLUE,2);

	putText(frame,intToString(y),Point(x+2,y-42),1,1,BLUE,2);
	putText(frame,intToString(x),Point(x+40,y+10),1,1,RED,2);

}

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	
	//release
	erodeElement.release();
	dilateElement.release();

}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);

	//these two vectors needed for output of findContours
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObjectV2(x,y,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}

	//release memory
	temp.release();

}