#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	VideoCapture cap(0);//enter cam # or video location
	if (!cap.isOpened()) return -1; //check if succeeded

	namedWindow("frame", WINDOW_AUTOSIZE);//captured img
	namedWindow("control", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("frameThresh", WINDOW_AUTOSIZE);//threshold aken

	//initial values for trackbars
	int lowH = 0;
	int highH = 25;
	
	int lowS = 0;
	int highS = 255;

	int lowV = 0;
	int highV = 255;

	//trackbar creation
	cvCreateTrackbar("LowH", "control", &lowH, 179);//hue
	cvCreateTrackbar("HighH", "control", &highH, 179);

	cvCreateTrackbar("LowS", "control", &lowS, 255);//saturation
	cvCreateTrackbar("HighS", "control", &highS, 255);

	cvCreateTrackbar("LowV", "control", &lowV, 255);//value
	cvCreateTrackbar("HighV", "control", &highV, 255);

	for (;;) {
		Mat frame;//frame
		cap >> frame;
		if (!cap.read(frame)) break;//check for error

		Mat frameHSV;//frame in HSV
		cvtColor(frame, frameHSV, COLOR_BGR2HSV);//to HSV color space

		Mat frameThresh;//frame post thresh
		inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameThresh);//threshold img in ranges

		imshow("frame", frame);
		imshow("frameThresh", frameThresh);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break
		
	}

	destroyWindow("frame");
	return 0;
}
