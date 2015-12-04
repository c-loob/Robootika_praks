#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv\cv.h>
#include <image_proc.h>
using namespace cv;
using namespace std;

//calculate euclidean distance between 2 pts
float eucl_dist(Point2f corner1, Point2f corner2){
	return (sqrt((corner1.x - corner2.x)*(corner1.x - corner2.x) + (corner1.y - corner2.y)*(corner1.y - corner2.y)));
}

//get thersholded, eroded etc img
Mat preprocess(Mat frame, int lowH, int lowS, int lowV, int highH, int highS, int highV) {
	Mat thresh, HSV;
	cvtColor(frame, HSV, COLOR_BGR2HSV);//to HSV color space
	inRange(HSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), thresh);

	erode(thresh, thresh, Mat(), Point(-1, -1), 2);
	dilate(thresh, thresh, Mat(), Point(-1, -1), 2);
	return thresh;
}

//get mass center of goal(mc of biggest contour)
tuple<Point2f, Point2f, Point2f> process_goal(vector<vector<Point>> contours, Mat frame, Scalar varv) {
	float biggest_contour_area = 0;
	int biggest_contour_id = -1;
	vector<Point2f> mc_goal(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
		float ctArea = contourArea(contours[i]);
		if ((ctArea > biggest_contour_area) && (ctArea>500)) {
			biggest_contour_area = ctArea;
			biggest_contour_id = i;
		}
	}

	if ((biggest_contour_id < 0)) {
		Point2f temp, temp2, temp3;
		temp.x = -1;
		temp.y = -1;
		temp2.x = -1;
		temp2.y = -1;
		temp3.x = -1;
		temp3.y = -1;
		return make_tuple(temp, temp2, temp3);;//väravat ei leitud
	}
	else {
		vector<Moments> mu_goal(contours.size());
		mu_goal[biggest_contour_id] = moments(contours[biggest_contour_id], false);

		mc_goal[biggest_contour_id] = Point2f(mu_goal[biggest_contour_id].m10 / mu_goal

			[biggest_contour_id].m00, mu_goal[biggest_contour_id].m01 / mu_goal[biggest_contour_id].m00);

		circle(frame, mc_goal[biggest_contour_id], 4, Scalar(255, 0, 0), -1, 8, 0);

		RotatedRect boundingBox = minAreaRect(contours[biggest_contour_id]);
		Point2f corners[4];
		corners[0].x = -1;
		corners[0].y = 1;
		corners[1].x = -1;
		corners[1].y = 1;
		corners[2].x = -1;
		corners[2].y = 1;
		corners[3].x = -1;
		corners[3].y = 1;
		boundingBox.points(corners);

		float longest = -1;
		int longest_id = -1;
		if (eucl_dist(corners[0], corners[1]) > longest){
			longest_id = 0;
			longest = eucl_dist(corners[0], corners[1]);
		}
		if (eucl_dist(corners[1], corners[2]) > longest){
			longest_id = 1;
			longest = eucl_dist(corners[1], corners[2]);
		}
		if (eucl_dist(corners[2], corners[3]) > longest){
			longest_id = 2;
			longest = eucl_dist(corners[2], corners[3]);
		}
		if (eucl_dist(corners[3], corners[0]) > longest){
			longest_id = 3;
			longest = eucl_dist(corners[3], corners[0]);
		}
		line(frame, corners[0], corners[1], Scalar(255, 0, 0));
		line(frame, corners[1], corners[2], Scalar(255, 0, 0));
		line(frame, corners[2], corners[3], Scalar(255, 0, 0));
		line(frame, corners[3], corners[0], Scalar(255, 0, 0));

		if (longest_id == 0){
			line(frame, corners[0], corners[1], Scalar(0, 0, 255));
			return make_tuple(mc_goal[biggest_contour_id], corners[0], corners[1]);
		}
		else if (longest_id == 1){
			line(frame, corners[1], corners[2], Scalar(0, 0, 255));
			return make_tuple(mc_goal[biggest_contour_id], corners[1], corners[2]);
		}
		else if (longest_id == 2){
			line(frame, corners[2], corners[3], Scalar(0, 0, 255));
			return make_tuple(mc_goal[biggest_contour_id], corners[2], corners[3]);
		}
		else if (longest_id == 3){
			line(frame, corners[3], corners[0], Scalar(0, 0, 255));
			return make_tuple(mc_goal[biggest_contour_id], corners[3], corners[0]);
		}

		//return mc_goal[biggest_contour_id];
	}
}

//väljastab ainult kõige suurema kontuuri, raadiuse
tuple<Point2f, float> process_ball(vector<vector<Point>> contours, Mat frame) {

	vector<Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}
	//get mass centers
	vector<Point2f> mc(contours.size());
	if (contours.size() > 0) {
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		Mat imgDrawing = Mat::zeros(frame.size(), CV_8UC3);

		for (int i = 0; i < contours.size(); i++) {
			if (contourArea(contours[i])>100) {
				//drawContours(frame, contours_ball, i, Scalar(0, 0, 255), 2, 8, hierarchy_ball, 0, Point());
				//circle(frame, mc[i], 4, Scalar(255, 0, 0), -1, 8, 0);
				float radius(contours[i].size());
			}
		}
		float biggest_contour_area = 0;
		int biggest_contour_id = 0;

		for (int i = 0; i < contours.size(); i++) {
			//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
			float ctArea = contourArea(contours[i]);
			if (ctArea > biggest_contour_area) { //if below line
				circle(frame, Point(320, 480), 4, Scalar(0, 0, 0), 1, 8, 0);
				/*
				Mat temp, temp2, temp3 = Mat::zeros(frame.size(), CV_8UC3);
				line(temp, a, b, Scalar(255, 255, 255), 4, 8, 0);
				line(temp2, Point(320, 480), mc[i], Scalar(255, 255, 255), 4, 8, 0);
				bitwise_and(temp, temp2, temp3);
				double min, max;
				int countw = 0;
				for (int y = 0; y < temp3.rows; y++){
				for (int x = 0; x < temp3.cols; x++){
				if (temp3.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0)){
				countw += 1;
				}

				}
				}
				if (countw > 0){
				cout << "found ball" << endl;
				float ball_dist = sqrt((mc[i].x - 320)*(mc[i].x - 320) + (mc[i].y - 480)*(mc[i].y - 480));

				biggest_contour_area = ctArea;
				biggest_contour_id = i;
				}
				}
				*/
				//if (a.x != -1){
				/*
				if (((((mc[i].x - a.x)*(b.y - a.y) - (mc[i].y - a.y)*(b.x - a.x))*((320 - a.x)*(b.y - a.y) - (480 - a.y)*(b.x - a.x))) < 0)
				&& ((((a.x - mc[i].x)*(480 - mc[i].y) - (a.y - mc[i].y)*(320 - mc[i].x))*((b.x - mc[i].x)*(480 - mc[i].y) - (b.y - mc[i].y)*(320 - mc[i].x))) < 0)){
				//cout << "found ball" << endl;
				float ball_dist = sqrt((mc[i].x - 320)*(mc[i].x - 320) + (mc[i].y - 480)*(mc[i].y - 480));

				biggest_contour_area = ctArea;
				biggest_contour_id = i;
				}

				else{
				float ball_dist = sqrt((mc[i].x - 320)*(mc[i].x - 320) + (mc[i].y - 480)*(mc[i].y - 480));

				biggest_contour_area = ctArea;
				biggest_contour_id = i;
				//}
				}*/
				/*
				else if (((((mc[i].x - a.x)*(b.y - a.y) - (mc[i].y - a.y)*(b.x - a.x))*((320 - a.x)*(b.y - a.y) - (480 - a.y)*(b.x - a.x))) > 0)
				&& ((((a.x - mc[i].x)*(480 - mc[i].y) - (a.y - mc[i].y)*(320 - mc[i].x))*((b.x - mc[i].x)*(480 - mc[i].y) - (b.y - mc[i].y)*(320 - mc[i].x))) > 0)){
				cout << "found ball" << endl;
				float ball_dist = sqrt((mc[i].x - 320)*(mc[i].x - 320) + (mc[i].y - 480)*(mc[i].y - 480));

				biggest_contour_area = ctArea;
				biggest_contour_id = i;
				}*/
			}

		}

		float r = sqrt(biggest_contour_area / 3.1415);
		//cout << "r= " << biggest_contour_area;
		if (biggest_contour_id != -1){
			circle(frame, mc[biggest_contour_id], r, Scalar(255, 0, 0), -1, 8, 0);
		}
		else{
			mc[biggest_contour_id].x = -1;
			mc[biggest_contour_id].y = -1;
		}
		return make_tuple(mc[biggest_contour_id], r);
	}
	else {
		//kui ühtegi palli ei leita, siis tagastatakse koordinaadid (-1, -1)
		Point2f temp;
		temp.x = -1;
		temp.y = -1;
		float temp2 = 0;
		return make_tuple(temp, temp2);
	}
}