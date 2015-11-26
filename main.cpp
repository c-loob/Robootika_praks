/*
NB!
liikumine toimub vektori abil kus:
esimene element tähistab liikumist y teljel(otse)
teine liikumist x teljel(külgedele).
kolmas roteerumist(positiivne = vastupäeva või vasakule, negatiivne = päripäeva või paremale)
liigutamiseks tuleb defineerida:
float liigu[3] = { 0, 0, 1 };
int max_speed = ?;
ning liikumise saab esile kutsuda:
movement(liigu, max_speed);
*/

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>
#include <vector>
#include <opencv\cv.h>
#include <stdio.h>
#include <time.h>//sleepcp - windows/posix sys-s? possible future error
#include <serial\serial.h>
#include <thread>
#include <string>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <future>
#include <tuple>
#include <boost\asio.hpp>
#include <boost\asio\serial_port.hpp>
#include <boost\system\error_code.hpp>
#include <boost\system\system_error.hpp>
#include <boost\bind.hpp>
#include <boost\thread.hpp>

using namespace cv;
using namespace std;

class Serial{
	char read_msg_[512];
	boost::asio::io_service m_io;
	boost::asio::serial_port m_port;

private:
	void handler(const boost::system::error_code& error, size_t bytes_transferred){
		read_msg_[bytes_transferred] = 0;
		std::cout << bytes_transferred << " bytes: " << read_msg_ << std::endl;

		read_some();
	}

	void read_some(){

		m_port.async_read_some(boost::asio::buffer(read_msg_, 512),
			boost::bind(&Serial::handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	}

public:
	Serial(const char *dev_name) :m_io(), m_port(m_io, dev_name){
		read_some();

		boost::thread t(boost::bind(&boost::asio::io_service::run, &m_io));
	}
};



//global stuff
std::mutex mu;
std::mutex mu2;
std::condition_variable cond;
vector< vector<Point> > contours_ball, contours_goal1, contours_goal2, contours_black;
vector< Vec4i > hierarchy_ball, hierarchy_goal, hierarchy_black;

char bl_det[];
char bl;
bool dribbler;
bool suund;

//sihtimise limiidid
int vasak_limiit =245;
int parem_limiit = 395;

int vasak_limiitG = 200;
int parem_limiitG = 440;

//initial values for trackbars
int G_lowH2 = 90;
int G_highH2 =136;
int G_lowS2 = 7;
int G_highS2 = 150;
int G_lowV2 =4;
int G_highV2 = 200;

//Yellow goal
int G_lowH1 = 15;
int G_highH1 = 40;
int G_lowS1 = 100;
int G_highS1 = 2255;
int G_lowV1 = 70;
int G_highV1 = 255;

int B_lowH = 5;
int B_highH = 25;
int B_lowS = 80;
int B_highS = 255;
int B_lowV = 50;
int B_highV = 255;

//headers
void sleepcp(int milliseconds);
void move_robot(int * kiirus);

void movement(float liigu[3], int max_speed);
int ymarda(float a);
int * get_speed(float * joud);
float * move_vector(float liigu[3]);
int ymarda(float a);
void stop();
String receive(String port, int length);

String trrx();
void set_dribbler(int speed);
void stop_dribbler();
void charge();
void discharge();
void kick();
void move_robot(int * kiirus);
void ball_in(Point2f mc_goal);
void no_ball(Point2f mc_ball);
void tx(String command);

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

		mc_goal[biggest_contour_id] = Point2f(mu_goal[biggest_contour_id].m10 / mu_goal

			[biggest_contour_id].m00, mu_goal[biggest_contour_id].m01 / mu_goal[biggest_contour_id].m00);

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

//väljastab ainult kõige suurema kontuuri, raadiuse
pair<Point2f, float> process_ball(vector<vector<Point>> contours, Mat frame) {
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
		int biggest_contour_id = -1;

		for (int i = 0; i < contours.size(); i++) {
			//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
			float ctArea = contourArea(contours[i]);
			if (ctArea > biggest_contour_area) {
				biggest_contour_area = ctArea;
				biggest_contour_id = i;
			}
		}
		float r = sqrt(biggest_contour_area / 3.1415);
		circle(frame, mc[biggest_contour_id], r, Scalar(255, 0, 0), -1, 8, 0);
		return make_pair(mc[biggest_contour_id], r);
	}
	else {
		//kui ühtegi palli ei leita, siis tagastatakse koordinaadid (-1, -1)
		Point2f temp;
		temp.x = -1;
		temp.y = -1;
		float temp2 = 0;
		return make_pair(temp, temp2);
	}
}

void parse(){//check if in dribbler
	for (;;){

		String temp2 = trrx();
		if (temp2.length() == 8){//if correct
			bl = temp2[6];
		}
		//sleepcp(25);
	}
}

void tx(String command){
	String port = "COM3";
	//cout << "wat" << endl;
	//while (true){
	//unique_lock<mutex> lk(mu2);
	while (1){
		try {
			//mu.lock();
			//lk.lock();
			serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(20));
			
			if (my_serial.isOpen()) {
				
				//lk.lock();
				my_serial.write(command + "\r\n");
				//mu.unlock();
				cout << "done" << endl;
				break;
				
			}
			
			else{
				cout << "nope" << endl;
			}
			//lk.unlock();
		}
		catch (exception &e) {
			//cout << "error" << endl;
		}
	}
	//}
		
}
	
	


String trrx(){
	String port = "COM3";
	String command = "bl";
	try {

		serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(20));
		if (my_serial.isOpen()) {
			mu.lock();
			my_serial.write(command + "\n");
			String result = my_serial.read(8);
			//cout << result << endl;
			mu.unlock();
			return result;

		}
		else{
			String result = "+";
			return result;

		}


	}
	catch (exception &e) {
	}
}

String receive(String port, int length) {
	try {
		String result;

		serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(20));
		if (my_serial.isOpen()) {
			mu.lock();

			result = my_serial.read(length);//length - saadava str pikkus bytedes
			mu.unlock();
			return result;
		}
		else {
			result = "";
			return result;// not open
		}
	}
	catch (exception &e) {
		//cerr << "unhandeled exception: " << e.what() << endl;
		return "";
	}
}

float * move_vector(float liigu[3]){//liigu[3] = liikumise vektor {x, y, w} w-nurkkiirendus, 0 kui ei taha pöörata
		//liikumise maatriks, et ei peaks iga kord arvutama
		float liikumine[3][3] = { { 0.57735, -0.33333, 0.33333 }, { -0.57735, -0.33333, 0.33333 }, {

			0, 0.66667, 0.33333 } };
		float f1, f2, f3, x, y, w;
		static float tagastus[3];//jõudude vektor mille pärast tagastame

		x = liigu[0];
		y = liigu[1];
		w = liigu[2];

		//arvutame iga mootori jõu
		f1 = liikumine[0][0] * x + liikumine[0][1] * y + liikumine[0][2] * w;
		f2 = liikumine[1][0] * x + liikumine[1][1] * y + liikumine[1][2] * w;
		f3 = liikumine[2][0] * x + liikumine[2][1] * y + liikumine[2][2] * w;

		tagastus[0] = f1;
		tagastus[1] = f2;
		tagastus[2] = f3;

		return tagastus;
}

int * get_speed(float * joud, int max_speed){
	static int tagastus[3];

	tagastus[0] = ymarda(joud[0] * max_speed);
	tagastus[1] = ymarda(joud[1] * max_speed);
	tagastus[2] = ymarda(joud[2] * max_speed);

	return tagastus;
}

int ymarda(float a){
	if (a > 0){
		int b = (int)(a + 0.5);
		return (int)b;
	}
	else if (a < 0){
		int b = (int)(a - 0.5);
		return (int)b;
	}
	else return 0;
}

void movement(float liigu[3], int max_speed){
	float *jouvektor;
	jouvektor = move_vector(liigu);

	int *kiirused;
	kiirused = get_speed(jouvektor, max_speed);//!!!!!!!!!!!!!!!!!!!!!!
	//std::unique_lock<mutex> locker(mu);
	//cond.wait(locker);

	move_robot( kiirused);
	

	//locker.unlock();
}
/*
vector<String> to_vector(String cmd){
	vector<String> vektor;
	vektor.push_back(cmd);
	return vektor;
}

vector<String> to_vector(String cmd0, String cmd1){
	vector<String> vektor;
	vektor.push_back(cmd0);
	vektor.push_back(cmd1);
	return vektor;
}

vector<String> to_vector(String cmd0, String cmd1, String cmd2){
	vector<String> vektor;
	vektor.push_back(cmd0);
	vektor.push_back(cmd1);
	vektor.push_back(cmd2);
	return vektor;
}

*/
void stop(){
	float liigu[3] = { 0, 0, 0 };
	int speed = 0;
	movement(liigu, speed);
}

void set_dribbler(){
	if (!dribbler){
		tx("dm200");//!!!!!!!!!!!!!!!!!!!!!!!!!dm200 peab olema
		
		cout << "start" << endl;
		dribbler = true;
	}
}

void stop_dribbler(){
	if (dribbler){
		thread tsend2(tx, "dm0");
		tsend2.detach();
		cout << "stop" << endl;
		dribbler = false;
	}
}

void charge(){
	tx("c");
}

void discharge(){
	for (int i = 0; i < 20; i++){
		kick();
	}
}

void kick(){
	tx("k");
}

void move_robot(int * kiirus){//PRODUCER
	//NB 3 ja 1 mootori id hetkel vahetuses, sellepärast antakse 1. mootori kiirus kolmandale jms
	String port = "COM3";
	String cmd1 = "3:sd" + to_string(kiirus[1]) + "\r\n" + "2:sd" + to_string(kiirus[2]) + "\r\n" + "1:sd" + to_string(kiirus[0]) + "\r\n";
	thread test(tx,cmd1);
	test.join();
	//thread test(tx, cmd1);
	//test.detach();
	//sleepcp(100);
	
	//tsend.detach();
	//String cmd2 = "2:sd" + to_string(kiirus[2])+"\r\n";
	//tx(cmd2);
	//thread tsend2(tx, cmd2);
	//tsend2.detach();
	//String cmd3 = "1:sd" + to_string(kiirus[0])+"\r\n";
	///tx(cmd3);
	//thread tsend3(tx, cmd3);
	//tsend3.detach();
	//stop();
}

void ball_in(Point2f mc_goal){//ball in dribbler
	
	if (mc_goal.x != -1){//1. kiirus

		if (mc_goal.x < vasak_limiit){//pöörame vasakule(1
			if (suund == true){
				float liigu[3] = { 0, 0, 0.2 };
				movement(liigu, 45);
			}
			else{
				float liigu[3] = { 0, 0, -0.3 };//vasakule
				movement(liigu, 35);
			}
			suund = false;
		}
		else if (mc_goal.x > parem_limiit){
			if (suund == false){
				float liigu[3] = { 0, 0, 0.2 };
				movement(liigu,45);
			}
			else{
				float liigu[3] = { 0, 0, -0.3 };
				movement(liigu, 35);
			}
			suund = true;
		}
		else if ((mc_goal.x<parem_limiit)&&(mc_goal.x>vasak_limiitG)){
			tx("c\r\n");
			sleepcp(1000);
			tx("k\r\n");
		}
	}
	else{
		float liigu[3] = { 0, 0.2, 0.3 };
		movement(liigu, 35);
	}
	/*set_dribbler();
	int speed = 75;
	if (mc_goal.x != -1){//värav vaateväljas
		if (mc_goal.x < vasak_limiitG){//pöörame vasakule(1)
			float liigu[3] = { 0, 0, 0.3 };
			movement( liigu, speed);
		}
		else if (mc_goal.x > parem_limiitG){//paremale
			//cout << "parem" << endl;
			float liigu[3] = { 0, 0, -0.3 };
			movement(liigu, speed);
			
		}
		else {
			stop();
			//charge();
			sleepcp(2000);
			kick();
		}
	}
	else{//SEARCH FOR GOAL
		float liigu[3] = { 0, 0, 0.3 };
		movement( liigu, speed);
		
	}*/
}

void no_ball(Point2f mc_ball, float kaugus){
	
	//int speed = 150;
	
	//cout << "nope" << endl;
	//keera palli suunale; pall on vaateväljas
	if (mc_ball.x != -1){//1. kiirus

		if (mc_ball.x < vasak_limiit){//pöörame vasakule(1
			if (suund == true){
				float liigu[3] = { 0, 0, -0.3 };
				movement(liigu, 50);
			}
			else{
				float liigu[3] = { 0, 0, 0.2 };//vasakule
				movement(liigu, 35);
			}
			suund =false;
		}
		else if (mc_ball.x > parem_limiit){
			if (suund == false){
				float liigu[3] = { 0, 0, 0.3 };
				movement(liigu, 50);
			}
			else{
				float liigu[3] = { 0, 0, -0.2 };
				movement(liigu, 35);
			}
			suund = true;
		}
		else{
			float liigu[3] = { 1, 0, 0 };
			movement(liigu, 45);
			suund = NULL;
		}
	}
	else{
		float liigu[3] = { 0, 0.2, 0.3 }; 
		movement(liigu, 35);
	}
		//sleepcp(100);
		//stop();
		/*
		if (kaugus < 150){
		set_dribbler();
		speed = 100;
		//cout << "1" << endl;
		}
		else{
		stop_dribbler();
		speed = 150;
		}
		if (mc_ball.x < vasak_limiit){//pöörame vasakule(1)//teine kiirus
		float liigu[3] = { 0, 0, 0.2 };
		if (suund = true){//eelmine paremale
		movement(liigu, speed / 4);
		}
		else{
		movement( liigu, speed);
		}
		suund = false;

		}
		else if (mc_ball.x > parem_limiit){//paremale
		cout << "parem" << endl;
		float liigu[3] = { 0, 0, -0.2 };
		if (suund = false){//eelmine vasakule
		movement(liigu, speed / 4);
		}
		else{
		movement( liigu, speed);
		}
		suund = true;
		}
		else {
		float liigu[3] = {0.5, 0, 0 };
		if (suund == NULL){//eelmine ka otse
		if (2 * speed > 250){
		speed = 200;
		}
		else{
		//nothing
		}
		movement(liigu, speed);
		}
		movement( liigu, 50);
		suund = NULL;

		//cout << "otse" << endl;
		}

		}

		else{//SEARCH FOR BALL
		float liigu[3] = { 0, 0, 0.3 };
		//thread t1(movement, liigu, speed);
		//t1.detach();
		}
		*/
	}
//}

tuple<Mat, Point2f, Point2f, float> get_frame(VideoCapture cap, String goal){
	Mat frame, pall_thresh, goal_thresh;
	cap >> frame;
	if (!cap.read(frame)) cout << "error reading frame" << endl;//check for error'

	//BALL
	Point2f mc_ball;//mass centers

	pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);

	findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	pair<Point2f, float> result = process_ball(contours_ball, frame);
	mc_ball = result.first;
	float raadius = result.second;

	//arvutame palli !umbkaudse! kauguse kaamerast
	//päris palli suurus * käsitsi leitud fokaalpikkuse parameeter/palli diameeter pikslites
	float palli_kaugus = 2.5 * 1060 / (2 * raadius);
	putText(frame, (to_string(palli_kaugus) + "cm"), cvPoint(30, 60),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
	goal = "yellow";
	//GOAL
	if (goal == "yellow"){
		goal_thresh = preprocess(frame, G_lowH1, G_lowS1, G_lowV1, G_highH1, G_highS1, G_highV1);
		
	}
	if(goal == "blue"){
		goal_thresh = preprocess(frame, G_lowH2, G_lowS2, G_lowV2, G_highH2, G_highS2, G_highV2);
	}
	imshow("goal",goal_thresh);
	waitKey(30);
	//imshow("goal", goal_thresh);
	//waitKey(30);
	findContours(goal_thresh, contours_goal1, hierarchy_goal, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Point2f mc_goal = process_goal(contours_goal1, frame, Scalar(255, 255, 255));
	
	//black lines
	/*
	Mat black_thresh = preprocess(frame,0, 0, 0,255, 200, 70);
	vector<Vec2f> lines;
	HoughLines(black_thresh, lines, 1, CV_PI / 180, 150, 0, 0);
	for (size_t i = 0; i < lines.size(); i++){
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(frame, pt1, pt2, Scalar(0, 0, 0), 3, CV_AA);
	}
	*/
	return make_tuple(frame, mc_ball, mc_goal, palli_kaugus);
}


int main() {
	Mat frame;
	Point2f mc_ball, mc_goal;
	float kaugus;
	int speed = 150;
	
	tx("dm255");
	//trackbar creation
	namedWindow("control_goal1", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal2", WINDOW_AUTOSIZE);//trackbaride aken

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


	//thread to check if ball in dribbler
	thread t3(parse);
	t3.detach();

	VideoCapture cap(0);//enter cam # or video location-------------------
	if (!cap.isOpened()) return -1; //check if succeeded


	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	String goal = "yellow";
	for (;;) {
		//set_dribbler();
		tie(frame, mc_ball, mc_goal, kaugus) = get_frame(cap, goal);

		//float liigu[3] = { 1, 0, 0 };
		//(liigu, 75);
		
		if (bl == '1'){
			ball_in(mc_goal);
		}
		else if (bl == '0'){
			no_ball(mc_ball, kaugus);
			
		}
		
		//float liigu[3] = { 1, 0, 0 };
		//movement(liigu, 75);
		imshow("orig", frame);
		waitKey(10);
	}


	//destroyAllWindows();
	return 0;
}