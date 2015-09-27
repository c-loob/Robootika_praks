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

	namedWindow("frame", WINDOW_AUTOSIZE);//captured img
	namedWindow("control", WINDOW_AUTOSIZE);//trackbaride aken

	//initial values for trackbars
	int lowH = 0;
	int highH = 25;

	int lowS = 84;
	int highS = 255;

	int lowV = 22;
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

	vector< vector<Point> > contours;
	vector< Vec4i > hierarchy;

	for (;;) {
		Mat frame;//frame
		cap >> frame;
		if (!cap.read(frame)) break;//check for error'
		
		//HSV threshold:
		Mat frameHSV, hsv_channels[3];//frame in HSV
		cvtColor(frame, frameHSV, COLOR_BGR2HSV);//to HSV color space
		Mat frameThresh;//frame post thresh
		inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameThresh);//threshold img in ranges

		erode(frameThresh, frameThresh, Mat(), Point(-1, -1), 2);
		dilate(frameThresh, frameThresh, Mat(), Point(-1, -1), 2);
		imshow("processed", frameThresh);

		//contours:
		findContours(frameThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		//get moments
		vector<Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}

		//get mass centers
		vector<Point2f> mc(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		Mat imgDrawing = Mat::zeros(frame.size(), CV_8UC3);

		for (int i = 0; i < contours.size(); i++) {
			if (contourArea(contours[i])>100) {
				drawContours(imgDrawing, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());
				circle(imgDrawing, mc[i], 4, Scalar(255, 0, 0), -1, 8, 0);
			}
		}

		imshow("frame", imgDrawing);
		imshow("orig", frame);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break

	}


	destroyAllWindows();
	return 0;
}
