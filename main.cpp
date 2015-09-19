#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>
#include <vector>


using namespace cv;
using namespace std;

int main() {
	VideoCapture cap("H:\\var\\pall.mp4");//enter cam # or video location
	if (!cap.isOpened()) return -1; //check if succeeded

	cap.set(CAP_PROP_FRAME_WIDTH, 640);//feed resolution from webcam
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("frame", WINDOW_AUTOSIZE);//captured img
	namedWindow("control", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("frameThresh", WINDOW_AUTOSIZE);//threshold aken

	//initial values for trackbars
	int lowH = 0;
	int highH = 25;
	
	int lowS = 150;
	int highS = 255;

	int lowV = 100;
	int highV = 255;

	int hi = 100, lo = 80;

	//trackbar creation
	createTrackbar("hi", "control", &hi, 255);
	createTrackbar("lo", "control", &lo, 255);

	createTrackbar("LowH", "control", &lowH, 179);//hue
	createTrackbar("HighH", "control", &highH, 179);

	createTrackbar("LowS", "control", &lowS, 255);//saturation
	createTrackbar("HighS", "control", &highS, 255);

	createTrackbar("LowV", "control", &lowV, 255);//value
	createTrackbar("HighV", "control", &highV, 255);

	vector<Vec3f> circles;
	int erosion_size = 6;
	Mat element = getStructuringElement(MORPH_CROSS, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
	for (;;) {
		Mat frame;//frame
		cap >> frame;
		if (!cap.read(frame)) break;//check for error

		//HSV threshold:
		Mat frameHSV,hsv_channels[3];//frame in HSV
		cvtColor(frame, frameHSV, COLOR_BGR2HSV);//to HSV color space
		Mat frameThresh;//frame post thresh
		inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameThresh);//threshold img in ranges

		//houghcircles:
		Mat gray;
		erode(frameThresh, gray, element);
		dilate(gray, gray, element);
		GaussianBlur(gray, gray, Size(5, 5), 2, 2);
		
		HoughCircles(gray, circles, HOUGH_GRADIENT, 2.0, 20.0, hi>0?hi:1, lo >0?lo:1);

		//draw detected circles:
		for (size_t i = 0; i < circles.size(); i++) {
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
			circle(frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
			
		}


		//display:
		imshow("frame", frame);
		imshow("frameThresh", gray);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break
		
	}

	destroyWindow("frame");
	return 0;
}
