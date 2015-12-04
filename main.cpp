#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>
#include <vector>
#include <opencv\cv.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <iostream>
#include <boost\asio.hpp>
#include <boost\thread.hpp>
#include <mutex>
#include <image_proc.h>
#include <control_r.h>
#include <fstream>

using namespace cv;
using namespace std;

//global stuff
vector< vector<Point> > contours_ball, contours_goal1, contours_goal2, contours_black, contours_white;
vector< Vec4i > hierarchy_ball, hierarchy_goal, hierarchy_black, hierarchy_white;

bool dribbler;
bool suund;
mutex mu;
bool refstart = false;
bool stopbool = true;
bool bl = false;
String my_robotID = "A";
String my_field = "A";
bool goal_select = true; //true = yellow, false = blue
bool respond = false;

class SimpleSerial
{
public:
	/**
	* Constructor.
	* \param port device name, example "/dev/ttyUSB0" or "COM4"
	* \param baud_rate communication speed, example 9600 or 115200
	* \throws boost::system::system_error if cannot open the
	* serial device
	*/
	SimpleSerial(std::string port, unsigned int baud_rate)
		: io(), serial(io, port)
	{
		serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	}

	/**
	* Write a string to the serial device.
	* \param s string to write
	* \throws boost::system::system_error on failure
	*/
	void writeString(std::string s)
	{
		boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
	}

	/**
	* Blocks until a line is received from the serial device.
	* Eventual '\n' or '\r\n' characters at the end of the string are removed.
	* \return a string containing the received line
	* \throws boost::system::system_error on failure
	*/
	std::string readLine()
	{
		//Reading data char by char, code is optimized for simplicity, not speed
		using namespace boost;
		char c;
		std::string result;
		for (;;)
		{
			asio::read(serial, asio::buffer(&c, 1));
			switch (c)
			{
			case '\r':
				break;
			case '\n':
				return result;
			default:
				result += c;
			}
		}
	}

private:
	boost::asio::io_service io;
	boost::asio::serial_port serial;
};

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
		runner.join();
		
	
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
		async_read_until(port, buffer,
			"-",
			boost::bind(&SerialClass::onData,
			this, _1, _2));
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
			std::cout << data << endl;
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
			std::cout << data << endl;
			//cout <<"0 " <<  data[0] << " " << data[1] << " " << data[2] << endl;
			//if ((data[0] == 'a')){//kohtinuku käsuks piisavalt pikk ja algab 'a'-ga
				/*if ((data[1] == 'A') || (data[1] == 'X')){//command is for my field
					if ((data[2] == 'X') || (data[2] == 'A')){//command is for everybody
						if (data[2] == my_robotID[0]){
							respond = true;
						}*/
			if ((data.find("AX") != std::string::npos) || (data.find("AA") != std::string::npos) || (data.find("XX") != std::string::npos)){
						if (data.find("STOP") != std::string::npos){//command is STOP
							std::cout << "stopping..1." << endl;
							refstart = false;
							stopbool = true;
						}
						else{//command is START
						
							refstart = true;
							std::cout << "starting.1..." << endl;
							stopbool = false;
						}
			}
			if (data.find("STOP") != std::string::npos){//command is STOP
				std::cout << "stopping..." << endl;
				refstart = false;
				stopbool = true;
				std::cout << "stop" << endl;
			}
			else if (data.find("STAR") != std::string::npos){//command is START

				refstart = true;
				std::cout << "starting...." << endl;
				stopbool = false;
				std::cout << "start" << endl;
			}
					//}
				//}
			//}
			}
			std::cout << data << endl;
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
	boost::thread runner2;
	boost::asio::streambuf buffer;
	//boost::asio::streambuf buffer2;
	bool quitFlag;
};

//headers
void move_robot(int * kiirus, SerialClass& serial);
void movement(float liigu[3], int max_speed, SerialClass& serial);
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
/*
tuple<Mat, Point2f, Point2f, float> get_frame(VideoCapture cap, vector<int> ball, vector<int> yellow, vector<int> blue, int state){
	if (state == 1){//competition mode
		Mat frame, pall_thresh, goal_thresh, black_thresh, black_result, white_thresh, white_result;
		cap >> frame;
		if (!cap.read(frame)) std::cout << "error reading frame" << endl;//check for error'

		//BALL
		Point2f mc_ball;//mass centers

		pall_thresh = preprocess(frame, ball[0], ball[1], ball[2], ball[3], ball[4], ball[5]);

		findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		black_thresh = preprocess(frame, 0, 0, 0, 180, 255, 160);
		white_thresh = preprocess(frame, 0, 0, 240, 180, 255, 255);

		dilate(white_thresh, white_thresh, Mat(), Point(-1, -1), 10);
		bitwise_and(white_thresh, black_thresh, white_result);

		findContours(white_result, contours_white, hierarchy_white, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		Point2f mc_black, mc_white, corner1, corner2;
		tie(mc_black, corner1, corner2) = process_goal(contours_black, frame, Scalar(0, 0, 0));
		tie(mc_white, corner1, corner2) = process_goal(contours_white, frame, Scalar(255, 0, 0));

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
		if (goal_select == true){//yellow
			goal_thresh = preprocess(frame, yellow[0], yellow[1], yellow[2], yellow[3], yellow[4], yellow[5]);

		}
		if (goal_select == false){//blue
			goal_thresh = preprocess(frame, blue[0], blue[1], blue[2], blue[3], blue[4], blue[5]);
		}

		findContours(goal_thresh, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Point2f mc_goal, lamp0, lamp1;
		tie(mc_goal, lamp0, lamp1) = process_goal(contours_goal1, frame, Scalar(255, 255, 255));
		return make_tuple(frame, mc_ball, mc_goal, palli_kaugus);
	}
	else{
		Mat frame, pall_thresh, goal_thresh, black_thresh, black_result, white_thresh, white_result;
		cap >> frame;
		if (!cap.read(frame)) std::cout << "error reading frame" << endl;//check for error'

		//BALL
		Point2f mc_ball;//mass centers

		pall_thresh = preprocess(frame, ball[0], ball[1], ball[2], ball[3], ball[4], ball[5]);
		imshow("ball", pall_thresh);
		waitKey(10);

		findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		black_thresh = preprocess(frame, 0, 0, 0, 180, 255, 160);
		white_thresh = preprocess(frame, 0, 0, 240, 180, 255, 255);

		dilate(white_thresh, white_thresh, Mat(), Point(-1, -1), 10);
		bitwise_and(white_thresh, black_thresh, white_result);

		findContours(white_result, contours_white, hierarchy_white, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		Point2f mc_black, mc_white, corner1, corner2;
		tie(mc_black, corner1, corner2) = process_goal(contours_black, frame, Scalar(0, 0, 0));
		tie(mc_white, corner1, corner2) = process_goal(contours_white, frame, Scalar(255, 0, 0));

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

	
		goal_thresh = preprocess(frame, yellow[0], yellow[1], yellow[2], yellow[3], yellow[4], yellow[5]);
		imshow("yellow", goal_thresh);
		waitKey(10);
		goal_thresh = preprocess(frame, blue[0], blue[1], blue[2], blue[3], blue[4], blue[5]);
		imshow("blue", goal_thresh);
		waitKey(10);

		findContours(goal_thresh, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Point2f mc_goal, lamp0, lamp1;
		tie(mc_goal, lamp0, lamp1) = process_goal(contours_goal1, frame, Scalar(255, 255, 255));
		return make_tuple(frame, mc_ball, mc_goal, palli_kaugus);
	}
	
}
*/
//check if goal in view
tuple<Mat, Point2f> get_frame_goal(VideoCapture cap, vector<int> goal){
	Mat frame, goal_thresh;
	cap >> frame;
	if (!cap.read(frame)) std::cout << "error reading frame" << endl;//check for error'

	goal_thresh = preprocess(frame, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
	findContours(goal_thresh, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Point2f mc_goal, corner0, corner1;

	tie(mc_goal, corner0, corner1) = process_goal(contours_goal1, frame, Scalar(255, 255, 255));
	return make_tuple(frame, mc_goal);
}

//check if ball in view
tuple<Mat, Point2f> get_frame_ball(VideoCapture cap, vector<int> ball){
	Mat frame, ball_thresh;
	cap >> frame;
	if (!cap.read(frame)) std::cout << "error reading frame" << endl;//check for error'

	ball_thresh = preprocess(frame, ball[0], ball[1], ball[2], ball[3], ball[4], ball[5]);
	findContours(ball_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Point2f mc_ball;
	float kaugus;
	tie(mc_ball, kaugus) = process_ball(contours_ball, frame);
	return make_tuple(frame, mc_ball);
}

int main() {
	int state = 0;//select state 0-calib color, no serial; 1- competition mode, no trackbars
	Mat frame;
	Point2f mc_ball, mc_goal;
	float kaugus;
	int speed = 150;

	ifstream calib_param;
	calib_param.open("C:\\Users\\Dell\\Documents\\GitHub\\Robootika_praks\\calib_param.txt");
	char output[10];
	
	std::vector<int> ball_calib;
	std::vector<int> yellow_calib;
	std::vector<int> blue_calib;
	if (calib_param.is_open()){
		for (int i = 0; i < 3; i++){//3 sets of calib
			for (int j = 0; j < 6; j++){//6 per set
				calib_param >> output;
				if (i == 0){
					ball_calib.push_back(stoi(output));
				}
				else if (i == 1){
					yellow_calib.push_back(stoi(output));
				}
				else if (i == 2){
					blue_calib.push_back(stoi(output));
				}
			}
		}
	}

	if (state == 0){
		//trackbar creation
		namedWindow("yellow", WINDOW_AUTOSIZE);//trackbaride aken
		namedWindow("blue", WINDOW_AUTOSIZE);//trackbaride aken
		namedWindow("ball", WINDOW_AUTOSIZE);//trackbaride aken

		createTrackbar("LowH", "yellow", &yellow_calib[0], 179);//hue
		createTrackbar("HighH", "yellow", &yellow_calib[1], 179);
		createTrackbar("LowS", "yellow", &yellow_calib[2], 255);//saturation
		createTrackbar("HighS", "yellow", &yellow_calib[3], 255);
		createTrackbar("LowV", "yellow", &yellow_calib[4], 255);//value
		createTrackbar("HighV", "yellow", &yellow_calib[5], 255);

		createTrackbar("LowH", "blue", &blue_calib[0], 179);//hue
		createTrackbar("HighH", "blue", &blue_calib[1], 179);
		createTrackbar("LowS", "blue", &blue_calib[2], 255);//saturation
		createTrackbar("HighS", "blue", &blue_calib[3], 255);
		createTrackbar("LowV", "blue", &blue_calib[4], 255);//value
		createTrackbar("HighV", "blue", &blue_calib[5], 255);

		createTrackbar("LowH", "ball", &ball_calib[0], 179);//hue
		createTrackbar("HighH", "ball", &ball_calib[1], 179);
		createTrackbar("LowS", "ball", &ball_calib[2], 255);//saturation
		createTrackbar("HighS", "ball", &ball_calib[3], 255);
		createTrackbar("LowV", "ball", &ball_calib[4], 255);//value
		createTrackbar("HighV", "ball", &ball_calib[5], 255);

		VideoCapture cap(0);
		if (!cap.isOpened()) return -1;
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

		for (;;) {
			//tie(frame, mc_ball, mc_goal, kaugus) = get_frame(cap, ball_calib, yellow_calib, blue_calib, state);
			//imshow("calibrate", frame);
			waitKey(10);
		}
	}
	else if (state ==1){
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

		SimpleSerial serialr("COM4", 19200);


		VideoCapture cap(0);
		if (!cap.isOpened()) return -1;
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


		cout << "My robot ID: " << my_robotID[0] << endl;
		cout << "My field ID: " << my_field[0] << endl;
		if (goal_select == true){
			cout << "My goal: " << "yellow" << endl;
		}
		else{
			cout << "My goal: " << "blue" << endl;
		}
		cout << "waiting for start command..." << endl;
		bool tribler = false;

		for (;;) {

			while (stopbool == true){
				Mat frame;
				Point2f wat;
				//tie(frame, wat) = get_frame(cap);
				//imshow("line", frame);
				waitKey(10);
			}

			if (respond == true){
				String temp = "a" + my_field + my_robotID + "ACK-----\r";
				respond = false;
			}

			serial.send("bl\r\n");

			if (bl == true){
				if (mc_goal.x == -1){
					//search goal
					/*
					turn max 6 times(360deg), if no goal found move on
					*/
					int countTurn = 0;
					while (countTurn < 6){
						//float *liigu = turn16();
						//get frame
						//check if goal now in view, if yes, break
						//if no ball in dribbler, break
					}
					/*
					check if line in the way
					move to better position
					*/

				}
				else{
					//handle shooting
					/*
					if shot clear shoot,
					else wait once for 2 sec, if no clear shot, move to better position
					*/
				}
			}
			else {
				if (mc_ball.x == -1){
					//search ball
					/*
					turn max 6 times(360deg), if no ball found move on
					*/

					//float *liigu = turn16();
				}
				else{
					//handle catching
					/*
					turn to ball
					if path clear, move to ball
					else find better position
					*/
				}
			}

			imshow("orig", frame);
			waitKey(10);
		}
	}

	return 0;
}