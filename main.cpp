#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>
#include <vector>
#include <opencv\cv.h>
#include <stdio.h>
#include <time.h>//sleepcp - windows/posix sys-s? possible future error
#include <serial\serial.h>

using namespace cv;
using namespace std;

void sleepcp(int milliseconds);

void sleepcp(int milliseconds) // cross-platform sleep function
{
	clock_t time_end;
	time_end = clock() + milliseconds * CLOCKS_PER_SEC / 1000;
	while (clock() < time_end)
	{
	}
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
Point2f process_goal(vector<vector<Point>> contours, Mat frame, Scalar varv) {
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
		Point2f temp;
		temp.x = -1;
		temp.y = -1;
		return temp;//väravat ei leitud
	}
	else {
		vector<Moments> mu_goal(contours.size());
		mu_goal[biggest_contour_id] = moments(contours[biggest_contour_id], false);

		mc_goal[biggest_contour_id] = Point2f(mu_goal[biggest_contour_id].m10 / mu_goal[biggest_contour_id].m00, mu_goal[biggest_contour_id].m01 / mu_goal[biggest_contour_id].m00);

		circle(frame, mc_goal[biggest_contour_id], 4, Scalar(255, 0, 0), -1, 8, 0);

		RotatedRect boundingBox = minAreaRect(contours[biggest_contour_id]);
		Point2f corners[4];
		boundingBox.points(corners);
		line(frame, corners[0], corners[1], varv);
		line(frame, corners[1], corners[2], varv);
		line(frame, corners[2], corners[3], varv);
		line(frame, corners[3], corners[0], varv);

		return mc_goal[biggest_contour_id];
	}
}
/*
vector<Point2f> process_ball(vector<vector<Point>> contours, Mat frame) {
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
				circle(frame, mc[i], 4, Scalar(255, 0, 0), -1, 8, 0);
			}
		}
		return mc;
	}
	else {
		return mc;
	}
}
*/

//väljastab ainult kõige suurema kontuuri
Point2f process_ball(vector<vector<Point>> contours, Mat frame) {
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
			}
		}
		float biggest_contour_area = 0;
		int biggest_contour_id = -1;
		
		for (int i = 0; i < contours.size(); i++) {
			//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
			float ctArea = contourArea(contours[i]);
			if (ctArea > biggest_contour_area) {
				biggest_contour_area = ctArea;
				biggest_contour_id = i;
			}
		}
		circle(frame, mc[biggest_contour_id], 4, Scalar(255, 0, 0), -1, 8, 0);
		return mc[biggest_contour_id];
	}
	else {
		Point2f temp;
		temp.x = -1;
		temp.y = -1;
		return temp;
	}
}

void transmit(String port, String command){
	try {
		serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(200));
		if (my_serial.isOpen()) {
			my_serial.write(command + "\n");
		}
	}
	catch (exception &e) {
		cerr << "unhandeled exception: " << e.what() << endl;

	}
}

String receive(String port, int length) {
	try {
		serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(200));
		if (my_serial.isOpen()) {
			String result;
			result = my_serial.read(length);//length - saadava str pikkus bytedes
			return result;
		}
		else {
			return NULL;// not open
		}
	}
	catch (exception &e) {
		cerr << "unhandeled exception: " << e.what() << endl;
		return NULL;
	}
}

void straigth(int speed, String port);
void left(int speed, String port);
void right(int speed, String port);
void stop(String port);

void left(int speed, String port) {//same speed, same direction
	String cmd;
	cmd = to_string(speed);
	stop(port);
	transmit(port, ("1:sd"+cmd));
	transmit(port, ("2:sd" + cmd));
	transmit(port, ("3:sd" + cmd));
}



void stop(String port) {//all 0
	transmit(port, ("1:sd0"));
	transmit(port, ("2:sd0"));
	transmit(port, ("3:sd0"));
}



void right(int speed, String port) {//same speed, same direction
	String cmd;
	cmd = to_string(speed);
	stop(port);
	transmit(port, ("1:sd-" + cmd));
	transmit(port, ("2:sd-" + cmd));
	transmit(port, ("3:sd-" + cmd));
}



void straigth(int speed, String port) {//NEGATIVE speed will not work!!!!!
	String cmd;
	cmd = to_string(speed);
	stop(port);
	//transmit(port, ("1:sd0\n\r" + "2:sd-" + cmd + "\n\r" + "3:sd" + cmd + "\n\r"));

	transmit(port, ("1:sd0"));
	transmit(port, ("2:sd-" + cmd));
	transmit(port, ("3:sd" + cmd));
}

int main() {
	VideoCapture cap(1);//enter cam # or video location
	if (!cap.isOpened()) return -1; //check if succeeded

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("control_ball", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal1", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal2", WINDOW_AUTOSIZE);//trackbaride aken

    //initial values for trackbars
	int G_lowH1 = 50;
	int G_highH1 = 70;
	int G_lowS1 = 50;
	int G_highS1 = 150;
	int G_lowV1 = 50;
	int G_highV1 = 200;

	int G_lowH2 = 72;
	int G_highH2 = 110;
	int G_lowS2 = 155;
	int G_highS2 = 237;
	int G_lowV2 = 50;
	int G_highV2 = 200;

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
	
	//communication const
	String port = "COM3";
	int baud = 19200;

	int state = 0;
	bool otse = false;

	//init FPS benchmark stuff
	//start/end times; fps; frame counter; seconds elapsed since start
	time_t start, end;
	double fps;
	int f_counter;
	double sec;
	//start the clock
	time(&start);


	for (;;) {
		//how much time passed
		time(&end);

		//calculate FPS
		++f_counter;
		sec = difftime(end, start);
		fps = f_counter / sec;
		//print
		cout << fps;

		Mat frame, pall_thresh, v2rav_thresh1, v2rav_thresh2;//frame
		Point2f mc_goal1, mc_goal2, mc_ball;//mass center of goal
		//vector<Point2f> mc_ball;
		cap >> frame;
		if (!cap.read(frame)) break;//check for error'

		Point2f temp1, temp2;
		temp1.x = 120;
		temp1.y = 375;
		circle(frame, temp1, 4, Scalar(255, 255, 255), -1, 8, 0);//otsasõidu piirid
		temp1.x = 540;
		circle(frame, temp1, 4, Scalar(255, 255, 255), -1, 8, 0);

		temp2.x = 240;
		temp2.y = 220;
		circle(frame, temp2, 4, Scalar(0, 0, 0), -1, 8, 0);//kauged piirid
		temp2.x = 400;
		circle(frame, temp2, 4, Scalar(0, 0, 0), -1, 8, 0);
		

		//STATE0 - NO BALL

		if (state == 0) {
			//ball in frame? fetch or search
			pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);
			findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			mc_ball = process_ball(contours_ball, frame);

			if ((mc_ball.x == -1) && (mc_ball.y == -1) && (otse ==true ))  {//no ball in frame
				//search
				sleepcp(2000);
				otse = false;
				right(5, port);
				//sleepcp(25);//sleep 25ms
				//stop(port);
				cout << "search";
			}
			else {// ball in frame; INCREASE STATE if catch ball!
				float x, y; //suurima palli x ja y koordinaadid
				x = mc_ball.x;
				y = mc_ball.y;
				if (y<temp1.y){//ei ole veel "käeulatuses"
					//liigu mc_ball_biggest peale
					if (x == 0) {//palli pole näha
						cout << "errrrror" << endl;
					}
					else if (x > 400) {//vaja paremale pöörata
						right(10, port);
						//sleepcp(25);
						//stop(port);
						cout << "paremale" << endl;
					}
					else if (x < 240) {//vaja vasakule pöörata
						left(10, port);
						//sleepcp(25);
						//stop(port);
						cout << "vasakule" << endl;
					}
					//else straigth
					else {
						straigth(30, port);
						//sleepcp(25);
						//stop(port);
						otse = true;
						cout << "otse" << endl;
					}
				}
				else {//pall on käe ulatuses, laiendame piire, sõidame otsa lihtsalt.
					if (x > 540) {//vaja paremale pöörata
						cout << "paremaleotsa" << endl;
						right(10, port);
						//sleepcp(25);
						//stop(port);
					}
					else if (x < 120) {//vaja vasakule pöörata
						left(10, port);
						//sleepcp(25);
						//stop(port);
						cout << "vasakuleotsa" << endl;
					}
					//else straigth
					else {
						straigth(50, port);
						//sleepcp(50);
						//stop(port);
						otse = true;
						cout << "otseotsa" << endl;
					}
				}
			}
		}

		//STATE1 - GOT BALL

		else if (state == 1) {
			//goal in frame? goal in range? search or shoot
			v2rav_thresh1 = preprocess(frame, G_lowH1, G_lowS1, G_lowV1, G_highH1, G_highS1, G_highV1);
			findContours(v2rav_thresh1, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			mc_goal1 = process_goal(contours_goal1, frame, Scalar(255, 0, 0));//blue

			if ((mc_goal1.x == -1) && (mc_ball.y == -1)) {//can't see goal1, search
				right(5, port);
				sleepcp(25);//sleep 25ms
				stop(port);
				cout << "search";
			}
			else {//DECREASE STATE after shooting
				//if in range - shoot; else liigu
				float x, y; //värava x ja y koordinaadid
				x = mc_goal1.x;
				y = mc_goal1.y;

				//liigu mc_goal1_biggest peale

				if (x == 0) {//väravat pole näha
					cout << "errrrror" << endl;
				}
				else if (x > 400) {//vaja paremale pöörata
					cout << "paremale" << endl;
				}
				else if (x < 240) {//vaja vasakule pöörata
					cout << "vasakule" << endl;
				}
				//else straigth
				else {
					cout << "otse" << endl;
				}

			}
		}


		/*
		//preprocessing
		pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);
		v2rav_thresh1 = preprocess(frame, G_lowH1, G_lowS1, G_lowV1, G_highH1, G_highS1, G_highV1);//goal #1
		v2rav_thresh2 = preprocess(frame, G_lowH2, G_lowS2, G_lowV2, G_highH2, G_highS2, G_highV2);//goal #2

																								   //contours:
		findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(v2rav_thresh1, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(v2rav_thresh2, contours_goal2, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		//get mass centers of goals & balls
		mc_goal1 = process_goal(contours_goal1, frame, Scalar(255, 0, 0));//blue
		mc_goal2 = process_goal(contours_goal2, frame, Scalar(0, 0, 255));//red goal
		mc_ball = process_ball(contours_ball, frame);

		
		

		//try to connect
		
		try {
			serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(200));
			if (my_serial.isOpen()) {
				//cout << "Great success!";
				//do stuff to move to ball
				//if rotation necessary
				String command;
				
				if (x == 0) {//palli pole näha
					cout << "pole näha" << endl;
				}
				else if((x-320) > 20){//vaja paremale pöörata
					cout << "paremale" << endl;
				}
				else if ((320 - x) > 20) {//vaja vasakule pöörata
					cout << "vasakuöe" << endl;
				}
				//else straigth
				else {
					cout << "otse" << endl;
				}
				//my_serial.write(command);
			}
			else {
				cout << "no.";
			}
			}
			catch (exception &e) {
			cerr << "unhandeled exception: " << e.what() << endl;
		}
		*/
		imshow("orig", frame);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break

	}


	//destroyAllWindows();
	return 0;
}