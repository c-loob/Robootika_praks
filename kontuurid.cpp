#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>
#include <vector>


using namespace cv;
using namespace std;
//get thersholded, eroded etc img
Mat preprocess(Mat frame, int lowH, int lowS, int lowV, int highH, int highS, int highV){
	Mat thresh, HSV;
	cvtColor(frame, HSV, COLOR_BGR2HSV);//to HSV color space
	inRange(HSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), thresh);
	
	erode(thresh, thresh, Mat(), Point(-1, -1), 2);
	dilate(thresh, thresh, Mat(), Point(-1, -1), 2);
	return thresh;
}
//get mass center of goal(mc of biggest contour)
Point2f process_goal(vector<vector<Point>> contours, Mat frame){
	float biggest_contour_area = 0;
	int biggest_contour_id = -1;
	vector<Point2f> mc_goal(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
		float ctArea = contourArea(contours[i]);
		if (ctArea > biggest_contour_area) {
			biggest_contour_area = ctArea;
			biggest_contour_id = i;
		}
	}

	if (biggest_contour_id < 0) {
		cout << "no goal found";
		return NULL;//v�ravat ei leitud
	}
	else {
		vector<Moments> mu_goal(contours.size());
		mu_goal[biggest_contour_id] = moments(contours[biggest_contour_id], false);

		mc_goal[biggest_contour_id] = Point2f(mu_goal[biggest_contour_id].m10 / mu_goal[biggest_contour_id].m00, mu_goal[biggest_contour_id].m01 / mu_goal[biggest_contour_id].m00);

		circle(frame, mc_goal[biggest_contour_id], 4, Scalar(255, 0, 0), -1, 8, 0);

		RotatedRect boundingBox = minAreaRect(contours[biggest_contour_id]);
		Point2f corners[4];
		boundingBox.points(corners);
		line(frame, corners[0], corners[1], Scalar(255, 255, 255));
		line(frame, corners[1], corners[2], Scalar(255, 255, 255));
		line(frame, corners[2], corners[3], Scalar(255, 255, 255));
		line(frame, corners[3], corners[0], Scalar(255, 255, 255));

		return mc_goal[biggest_contour_id];
	}
}

int main() {
	VideoCapture cap(0);//enter cam # or video location
	if (!cap.isOpened()) return -1; //check if succeeded
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("control_ball", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal", WINDOW_AUTOSIZE);//trackbaride aken

	//initial values for trackbars
	int G_lowH = 72;
	int G_highH = 110;
	int G_lowS = 155;
	int G_highS = 237;
	int G_lowV = 22;
	int G_highV = 255;

	int B_lowH = 0;
	int B_highH = 25;
	int B_lowS = 100;
	int B_highS = 237;
	int B_lowV = 22;
	int B_highV = 255;

	//trackbar creation
	createTrackbar("LowH", "control_ball", &B_lowH, 179);//hue
	createTrackbar("HighH", "control_ball", &B_highH, 179);
	createTrackbar("LowS", "control_ball", &B_lowS, 255);//saturation
	createTrackbar("HighS", "control_ball", &B_highS, 255);
	createTrackbar("LowV", "control_ball", &B_lowV, 255);//value
	createTrackbar("HighV", "control_ball", &B_highV, 255);

	createTrackbar("LowH", "control_goal", &G_lowH, 179);//hue
	createTrackbar("HighH", "control_goal", &G_highH, 179);
	createTrackbar("LowS", "control_goal", &G_lowS, 255);//saturation
	createTrackbar("HighS", "control_goal", &G_highS, 255);
	createTrackbar("LowV", "control_goal", &G_lowV, 255);//value
	createTrackbar("HighV", "control_goal", &G_highV, 255);

	vector< vector<Point> > contours_ball, contours_goal;
	vector< Vec4i > hierarchy_ball, hierarchy_goal;

	for (;;) {
		Mat frame, pall_thresh, v2rav_thresh;//frame
		Point2f mc_goal;//mass center of goal
		cap >> frame;
		if (!cap.read(frame)) break;//check for error'

		pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);
		v2rav_thresh = preprocess(frame, G_lowH, G_lowS, G_lowV, G_highH, G_highS, G_highV);

		//contours:
		findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(v2rav_thresh, contours_goal, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		//BALL PROCESSING
		//get moments
		vector<Moments> mu(contours_ball.size());
		for (int i = 0; i < contours_ball.size(); i++)
		{
			mu[i] = moments(contours_ball[i], false);
		}
		//get mass centers
		vector<Point2f> mc(contours_ball.size());
		for (int i = 0; i < contours_ball.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		Mat imgDrawing = Mat::zeros(frame.size(), CV_8UC3);

		for (int i = 0; i < contours_ball.size(); i++) {
			if (contourArea(contours_ball[i])>100) {
				drawContours(frame, contours_ball, i, Scalar(0, 0, 255), 2, 8, hierarchy_ball, 0, Point());
				circle(frame, mc[i], 4, Scalar(255, 0, 0), -1, 8, 0);
			}
		}

		//get mass center of goal
		mc_goal = process_goal(contours_goal, frame);

		imshow("orig", frame);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break

	}


	destroyAllWindows();
	return 0;
}
