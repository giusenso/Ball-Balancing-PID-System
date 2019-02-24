
#include "ball_tracker.h"
#include "ball_physic.h"

const char* path = "/home/jius/Desktop/ball-tracking-platform/ball_tracker/samples/test3.mp4";

/** @function main */
int main(int argc, char** argv){
  H_MIN = 48;
	H_MAX = 85;
	S_MIN = 88;
  S_MAX = 255;
	V_MIN = 65;
  V_MAX = 255;

  Point top_left, bottom_right;

  Ball* b = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
  printBall(b, 0);

  Mat src, threshold, HSV, box;

  VideoCapture cap;
  cap.open(path);

	cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  namedWindow( "img", CV_WINDOW_AUTOSIZE );
  namedWindow( "threshold", CV_WINDOW_AUTOSIZE );
  namedWindow( "box", CV_WINDOW_AUTOSIZE );

  while(1){

    cap.read(src);

    cvtColor(src,HSV,COLOR_BGR2HSV);

    inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

    morphOps(threshold);

    //prepare box
    //box = threshold(buildBox(b));

    trackFilteredObject(b, threshold, src);

    imshow( "img", src );
    imshow( "threshold", threshold );
    //imshow( "box", box );

    printBall(b, 0);

    if(waitKey(30) >= 0) break;

  }

  return 0;
}
