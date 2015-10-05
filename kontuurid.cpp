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
		return NULL;//väravat ei leitud
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
	namedWindow("control_goal1", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal2", WINDOW_AUTOSIZE);//trackbaride aken

	//initial values for trackbars
	int G_lowH1 = 72;
	int G_highH1 = 110;
	int G_lowS1 = 155;
	int G_highS1 = 237;
	int G_lowV1 = 22;
	int G_highV1 = 255;

	int G_lowH2 = 72;
	int G_highH2 = 110;
	int G_lowS2 = 155;
	int G_highS2 = 237;
	int G_lowV2 = 22;
	int G_highV2 = 255;

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

	createTrackbar("LowH", "control_goal1", &G_lowH1, 179);//hue
	createTrackbar("HighH", "control_goal1", &G_highH1, 179);
	createTrackbar("LowS", "control_goal1", &G_lowS1, 255);//saturation
	createTrackbar("HighS", "control_goal1", &G_highS1, 255);
	createTrackbar("LowV", "control_goal1", &G_lowV1, 255);//value
	createTrackbar("HighV", "control_goal1", &G_highV1, 255);

	createTrackbar("LowH", "control_goal2", &G_lowH2, 179);//hue
	createTrackbar("HighH", "control_goal2", &G_highH2, 179);
	createTrackbar("LowS", "control_goal2", &G_lowS2, 255);//saturation
	createTrackbar("HighS", "control_goal2", &G_highS2, 255);
	createTrackbar("LowV", "control_goal2", &G_lowV2, 255);//value
	createTrackbar("HighV", "control_goal2", &G_highV2, 255);

	vector< vector<Point> > contours_ball, contours_goal1, contours_goal2;
	vector< Vec4i > hierarchy_ball, hierarchy_goal;

	for (;;) {
		Mat frame, pall_thresh, v2rav_thresh1, v2rav_thresh2;//frame
		Point2f mc_goal1,mc_goal2;//mass center of goal
		cap >> frame;
		if (!cap.read(frame)) break;//check for error'

		//preprocessing
		pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);
		v2rav_thresh1 = preprocess(frame, G_lowH1, G_lowS1, G_lowV1, G_highH1, G_highS1, G_highV1);//goal #1
		v2rav_thresh2 = preprocess(frame, G_lowH2, G_lowS2, G_lowV2, G_highH2, G_highS2, G_highV2);//goal #2

		//contours:
		findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(v2rav_thresh1, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(v2rav_thresh2, contours_goal2, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		//get mass centers of goals
		mc_goal1 = process_goal(contours_goal1, frame);
		mc_goal2 = process_goal(contours_goal2, frame);

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

		imshow("orig", frame);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break

	}


	destroyAllWindows();
	return 0;
}
