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
#include <string>
#include <iostream>
#include <boost\asio.hpp>
#include <boost\thread.hpp>

using namespace cv;
using namespace std;

bool refstart = false;
bool stopbool = true;
bool bl = false;
String my_robotID = "A";
String my_field = "A";
//String goal = "yellow";
String goal = "blue";
bool respond = false;

class SerialClass{
public:
	SerialClass() :
		port(io),
		quitFlag(false){};

	~SerialClass()
	{
		//Stop the I/O services
		io.stop();
		
		//Wait for the thread to finish
		//runner.join();
		runner.detach();
	}

	bool connect(const std::string& port_name, int baud = 19200)
	{
		using namespace boost::asio;
		port.open(port_name);
		//Setup port
		port.set_option(serial_port::baud_rate(baud));
		port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));

		if ((port.is_open())&&(port_name.compare("COM4")))//if is open AND is NOT COM4
		{
			//Start io-service in a background thread.
			//boost::bind binds the ioservice instance
			//with the method call
			runner = boost::thread(
				boost::bind(
				&boost::asio::io_service::run,
				&io));

			startReceive();
		}
		else if ((port.is_open()) && (port_name.compare("COM3"))){
			runner = boost::thread(
				boost::bind(
				&boost::asio::io_service::run,
				&io));

			listenReferee();
		}
		

		return port.is_open();
	}

	void startReceive()
	{
		using namespace boost::asio;
		//Issue a async receive and give it a callback
		//onData that should be called when "\r\n"
		//is matched.
		async_read_until(port, buffer,
			"\n",
			boost::bind(&SerialClass::onData,
			this, _1, _2));
	}


	void listenReferee()
	{
		using namespace boost::asio;
		//Issue a async receive and give it a callback
		//onData that should be called when "\r\n"
		//is matched.
		//async_read_until(port, buffer,"-",boost::bind(&SerialClass::onData,this, _1, _2));
		async_read_until(port, buffer, "\r", boost::bind(&SerialClass::onData, this, _1, _2));
	}


	void send(const std::string& text)
	{
		boost::asio::write(port, boost::asio::buffer(text));
	}

	void send2(const std::string& text)
	{
		boost::asio::write(port, boost::asio::buffer(text));
	}

	void onData(const boost::system::error_code& e,
		std::size_t size)
	{

		if (!e)
		{
			std::istream is(&buffer);
			//std::istream is2(&buffer);
			std::string data(size, '\0');
			is.read(&data[0], size);
			
			if (((data.length()>6) && (data.compare("<4:bl:0>\n")==0)) || ((data.length()>6)&&(data.compare("<4:bl:1>\n")==0))){
				
				char bl_det = data[6];
				//cout << data<< endl;
				//cout << bl_det;
				if (bl_det == '0'){
					bl = false;
				}
				else if (bl_det == '1'){
					bl = true;
				}
			}
			else{
			cout << data << endl;
			//cout <<"0 " <<  data[0] << " " << data[1] << " " << data[2] << endl;
			if ((data[0] == 'a')){//kohtinuku käsuks piisavalt pikk ja algab 'a'-ga
				if ((data[1] == 'A') || (data[1] == 'X')){//command is for my field
					if ((data[2] == 'X') || (data[2] == 'A')){//command is for everybody
						if (data[2] == my_robotID[0]){
							respond = true;
						}
						if (data[5] == 'O'){//command is START
							cout << "stopping..." << endl;
							refstart = false;
							stopbool = true;
						}
						else{//command is STOP
						
							refstart = true;
							cout << "starting...." << endl;
							stopbool = false;
						}
					}
				}
			}
			}
			cout << data << endl;
			//If we receive quit()\r\n indicate
			//end of operations
			quitFlag = (data.compare("quit()\r\n") == 0);
		};

		startReceive();
	};

	bool quit(){ return quitFlag; }

private:
	boost::asio::io_service io;
	//boost::asio::io_service io2;
	boost::asio::serial_port port;

	boost::thread runner;
	//boost::thread runner2;
	boost::asio::streambuf buffer;
	//boost::asio::streambuf buffer2;
	bool quitFlag;
};

//global stuff
vector< vector<Point> > contours_ball, contours_goal1, contours_goal2, contours_black, contours_white;
vector< Vec4i > hierarchy_ball, hierarchy_goal, hierarchy_black, hierarchy_white;

bool dribbler;
bool suund;

//sihtimise limiidid
int vasak_limiit = 245;
int parem_limiit = 395;

int vasak_limiitG = 200;
int parem_limiitG = 440;

//initial values for trackbars
int G_lowH2 = 90;
int G_highH2 = 108;
int G_lowS2 = 160;
int G_highS2 = 255;
int G_lowV2 = 70;
int G_highV2 = 221;

//Yellow goal
int G_lowH1 = 15;
int G_highH1 = 40;
int G_lowS1 = 100;
int G_highS1 = 2255;
int G_lowV1 = 70;
int G_highV1 = 255;

//blue goal
int B_lowH = 5;
int B_highH = 25;
int B_lowS = 80;
int B_highS = 255;
int B_lowV = 50;
int B_highV = 255;

//headers
void sleepcp(int milliseconds);
void move_robot(int * kiirus);

void movement(float liigu[3], int max_speed, SerialClass& serial);
int ymarda(float a);
int * get_speed(float * joud);
float * move_vector(float liigu[3]);
int ymarda(float a);
void stop(bool stop, SerialClass& serial);
String receive(String port, int length);

void set_dribbler(int speed, SerialClass& serial);
void stop_dribbler(SerialClass& serial);
void charge(SerialClass& serial);
void discharge(SerialClass& serial);
void kick(SerialClass& serial);
void move_robot(int * kiirus, SerialClass& serial);
void ball_in(Point2f mc_goal, SerialClass& serial);
void no_ball(Point2f mc_ball, SerialClass& serial);
float eucl_dist(Point2f corner1, Point2f corner2);

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
tuple<Point2f, Point2f, Point2f> process_goal(vector<vector<Point>> contours, Mat frame, Scalar varv) {
	float biggest_contour_area = 0;
	int biggest_contour_id = -1;
	vector<Point2f> mc_goal(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
		float ctArea = contourArea(contours[i]);
		if ((ctArea > biggest_contour_area)&&(ctArea>500)) {
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
		return make_tuple(temp, temp2,temp3);;//väravat ei leitud
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

float eucl_dist(Point2f corner1, Point2f corner2){
	return (sqrt((corner1.x - corner2.x)*(corner1.x - corner2.x) + (corner1.y - corner2.y)*(corner1.y - corner2.y)));
}

//väljastab ainult kõige suurema kontuuri, raadiuse
pair<Point2f, float> process_ball(vector<vector<Point>> contours, Mat frame, Point2f a, Point2f b) {

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
				}
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

void movement(float liigu[3], int max_speed, SerialClass& serial){
	float *jouvektor;
	jouvektor = move_vector(liigu);

	int *kiirused;
	//leia mind
	kiirused = get_speed(jouvektor, max_speed);
	//kiirused = get_speed(jouvektor, 0);

	move_robot(kiirused, serial);

}

void stop(bool stop, SerialClass& serial){
	//freeze while stopped, if stop == false, continue
	while (stop){
		serial.send("dm0\r\n");
		float liigu[3] = { 0, 0, 0 };
		int speed = 0;
		serial.send("k\r\n");
		movement(liigu, speed, serial);
	}
	serial.send("dm250\r\n");
}

void move_robot(int * kiirus, SerialClass& serial){//PRODUCER
	String cmd1 = "3:sd" + to_string(kiirus[1]) + "\r\n" + "2:sd" + to_string(kiirus[2]) + "\r\n" + "1:sd" + to_string(kiirus[0]) + "\r\n";
	serial.send(cmd1);
}

void set_dribbler(int speed, SerialClass& serial){
	String cmd = "";
	if ((speed > 0) && (speed < 250)){
		cmd = ("dm" + to_string(speed) + "\r\n");
	}
	else if (speed > 250){
		cmd = ("dm250\r\n");
	}
	else{
		cmd = ("dm0\r\n");
	}
	serial.send(cmd);
}

void ball_in(Point2f mc_goal, SerialClass& serial){//ball in dribbler
	int hs = 150;
	int ms = 75;
	int ls = 50;

	if (mc_goal.x == -1){
		//search for ball
		float liigu[3] = { 0, -0.3, -0.5 };//paremale
		movement(liigu, ls, serial);
	}
	else if (mc_goal.x < 280){
		float liigu[3] = { 0, 0.3, 0.5 };//vasakule
		movement(liigu, ls, serial);
	}
	else if (mc_goal.x > 360){
		float liigu[3] = { 0, -0.3, -0.5 };//par4emale
		movement(liigu, ls, serial);
	}
	else if ((mc_goal.x > 279) && (mc_goal.x < 361)){
		float liigu[3] = { 0, 0, 0 };//par4emale
		movement(liigu, 0, serial);
		serial.send("c\r\n");
		sleepcp(1000);
		//set_dribbler(0, serial);
		serial.send("k\r\n");
		serial.send("c\r\n");
		//set_dribbler(255, serial);
		bl = false;
	}
}

void no_ball(Point2f mc_ball, float kaugus, SerialClass& serial){
	int hs = 75;
	int ms = 75;
	int ls = 50;
	/*
	hs = 0;
	ms = 0;
	ls = 0;
	*/
	if (mc_ball.x == -1){
		//search for ball
		float liigu[3] = { 0, -0.3, -0.5 };//paremale
		movement(liigu, ls, serial);
	}
	else if (mc_ball.x < 255){
		float liigu[3] = { 0, 0.3, 0.5 };//vasakule
		movement(liigu, ms, serial);
	}
	else if (mc_ball.x > 385){
		float liigu[3] = { 0, -0.3, -0.5 };//par4emale
		movement(liigu, ms, serial);
	}
	else if ((mc_ball.x > 254) && (mc_ball.x < 386)){
		float liigu[3] = { 1, 0, 0 };//otse
		if (kaugus > 150){
			movement(liigu, hs, serial);
		}
		else if (kaugus < 50){
			movement(liigu, ls, serial);
		}
		else{
			movement(liigu, ms, serial);
		}
	}
}

tuple<Mat, Point2f, Point2f, float> get_frame(VideoCapture cap, String goal){
	Mat frame, pall_thresh, goal_thresh, black_thresh, black_result, white_thresh, white_result;
	cap >> frame;
	if (!cap.read(frame)) cout << "error reading frame" << endl;//check for error'

	//BALL
	Point2f mc_ball;//mass centers

	pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);

	findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	black_thresh = preprocess(frame, 0, 0, 0, 180, 255, 160);
	white_thresh = preprocess(frame, 0, 0, 240, 180, 255, 255);
	
	dilate(white_thresh, white_thresh, Mat(), Point(-1, -1), 10);
	bitwise_and(white_thresh, black_thresh, white_result);

	imshow("test2", white_result);
	waitKey(10);

	findContours(white_result, contours_white, hierarchy_white, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	Point2f mc_black, mc_white, corner1, corner2;
	tie(mc_black, corner1, corner2) = process_goal(contours_black, frame, Scalar(0, 0, 0));
	tie(mc_white, corner1, corner2)= process_goal(contours_white, frame, Scalar(255, 0, 0));
	
	pair<Point2f, float> result = process_ball(contours_ball, frame, corner1, corner2);
	mc_ball = result.first;
	float raadius = result.second;

	//arvutame palli !umbkaudse! kauguse kaamerast
	//päris palli suurus * käsitsi leitud fokaalpikkuse parameeter/palli diameeter pikslites
	float palli_kaugus;
	if (raadius != 0){
		palli_kaugus = 2.5 * 1060 / (2 * raadius);
	}
	else{
		palli_kaugus = -1;
	}
	putText(frame, (to_string(palli_kaugus) + "cm"), cvPoint(30, 60),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
	
	//GOAL
	if (goal == "yellow"){
		goal_thresh = preprocess(frame, G_lowH1, G_lowS1, G_lowV1, G_highH1, G_highS1, G_highV1);

	}
	if (goal == "blue"){
		goal_thresh = preprocess(frame, G_lowH2, G_lowS2, G_lowV2, G_highH2, G_highS2, G_highV2);
	}
	
	findContours(goal_thresh, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	Point2f mc_goal, lamp0, lamp1;
	tie(mc_goal, lamp0, lamp1 ) = process_goal(contours_goal1, frame, Scalar(255, 255, 255));
	return make_tuple(frame, mc_ball, mc_goal, palli_kaugus);
}



int main() {
	Mat frame;
	Point2f mc_ball, mc_goal;
	float kaugus;
	int speed = 150;

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

	//connect to serial ports
	SerialClass serial;//control motors etc
	
	if (serial.connect("COM3", 19200))
	{
		std::cout << "Port COM3 is open." << std::endl;
	}
	else
	{
		std::cout << "Port open failed." << std::endl;
	}
	
	SerialClass serialref;//referee

	if (serialref.connect("COM4", 19200))
	{
		std::cout << "Port COM4 is open." << std::endl;
	}
	else
	{
		std::cout << "Port open failed." << std::endl;
	}
	
	VideoCapture cap(0);//enter cam # or video location-------------------
	if (!cap.isOpened()) return -1; //check if succeeded

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	serialref.send("+++");

	//serial.send("c\r\n");
	//serial.send("dm255\r\n");
	
	serial.send("c\r\n");
	//cout << "Referee test..." << endl;
	//serialref.send("+++");
	cout << "My robot ID: " << my_robotID[0] << endl;
	cout << "My field ID: " << my_field[0] << endl;
	cout << "My goal: " << goal << endl;
	//stopbool = false;
	bool tribler = false;
	for (;;) {
		
		if (stopbool == true){
			//cout << "stop" << endl;
			//stop(stopbool, serial);
			/*
			if (serialref.connect("COM4", 19200))
			{
				std::cout << "Port COM4 is open." << std::endl;
				serialref.send("+++");
			}
			else
			{
				std::cout << "Port open failed." << std::endl;
			}
			*/
			
		}
		else{
			//cout << "start" << endl;
			set_dribbler(0, serial);
			if (respond == true){
				String temp = "a" + my_field + my_robotID + "ACK-----\r";
				//serialref.send(temp);
				respond = false;
			}

			tie(frame, mc_ball, mc_goal, kaugus) = get_frame(cap, goal);
			if ((kaugus < 100) && (tribler == false) && (kaugus >0)){
				//set_dribbler(255, serial);
			}
			if (((kaugus > 100) && (tribler == true)) || (kaugus == -1) && (bl == false)){
				//set_dribbler(0, serial);
			}
			serial.send("bl\r\n");
			sleepcp(10);
			if (bl == true){
				ball_in(mc_goal, serial);
				//set_dribbler(200, serial);
				if (tribler == false){
					
					tribler = true;
				}
			}
			else {
				no_ball(mc_ball, kaugus, serial);
			}
			imshow("orig", frame);
			waitKey(10);
		}
	}


	//destroyAllWindows();
	return 0;
}