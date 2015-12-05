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
#include <math.h>

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
		try{
			if (!e)
			{

				std::istream is(&buffer);
				//std::istream is2(&buffer);
				std::string data(size, '\0');
				is.read(&data[0], size);
				//std::cout << data << endl;
				if (((data.length() > 6) && (data.compare("<4:bl:0>\n") == 0)) || ((data.length() > 6) && (data.compare("<4:bl:1>\n") == 0))){

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
				//	std::cout << data << endl;
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
				//std::cout << data << endl;
				//If we receive quit()\r\n indicate
				//end of operations
				quitFlag = (data.compare("quit()\r\n") == 0);
			};
		}
		catch (Exception &e){

		}
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
tuple<Mat, Point2f, float> get_frame_ball(Mat frame, vector<int> ball);
tuple<Mat, Point2f> get_frame_goal(Mat frame, vector<int> goal);
tuple<Mat, Point2f, Point2f> get_frame_line(Mat frame);
int tous(Point2f p1, Point2f p2);
void change_position_ball(Mat frame);
void find_ball(Mat frame, vector<int> ball, SerialClass& serial);
void aim_ball(Mat frame, vector<int> ball, SerialClass& serial);
void find_goal(Mat frame, vector<int> goal, SerialClass& serial);
void aim_goal(Mat frame, vector<int> goal, SerialClass& serial);
void turn16(bool direction, SerialClass& serial);
void turn(int speed, int direction, SerialClass& serial);
void otse(int speed, SerialClass& serial);

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

void set_dribbler(SerialClass& serial){
	String cmd = "";
	cmd = "dm150\r\n";
	serial.send(cmd);
	sleepcp(500);
	cmd = ("dm125\r\n");
	serial.send(cmd);
}

void stop_dribbler(SerialClass& serial){
	serial.send("dm0\r\n");
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

void stop_movement(SerialClass& serial){
	float liigu[3] = { 0, 0, 0 };
	movement(liigu, 0, serial);
}

void turn16(bool direction, SerialClass& serial){//direction 
	float turnamount =0.8;//pos == vasakule
	float strafe = 0.3;
	int speed = 40;
	float liigu[3] = { 0, 0, 0 };
	if (direction == true){
		liigu[1] = strafe;
		liigu[2] = turnamount ;
	}
	else{
		liigu[1] = -strafe;
		liigu[2] =-turnamount ;
	}
	movement(liigu, speed, serial);
	//sleepcp(2000);
}

void turn(int speed, int direction, SerialClass& serial){
	//sleepcp(100);
	//stop_movement(serial);//leiamind
	
	float liigu[3] = { 0, 0, 0 };
	if (direction == 1){//vasakule
		//cout << "test" << endl;
		liigu[2] = 0.5;
	}
	if(direction ==0){
		liigu[2] = -0.5;
	}
	movement(liigu, speed, serial);
}

void aim_goal(Mat frame, vector<int> goal, SerialClass& serial){
	
	Point2f mc;
	bool direction;
	while (true){
		get_frame_goal(frame, goal);

		if (mc.x == -1){
			break;
		}
		if (mc.x < 255){
			direction = true;
			turn(30, direction, serial);
		}
		else if (mc.x > 385){
			direction = false;
			turn(30, direction, serial);
		}
		else{

			break;
		}
		
	}
}

void find_goal(Mat frame, vector<int> goal, SerialClass& serial){
	stop_dribbler(serial);

	Point2f mc;

	while (mc.x == -1){
		for (int i = 0; i < 6; i++){//max 6 times
			tie(frame, mc) = get_frame_goal(frame, goal);
			if (mc.x != -1){
				break;
			}
			else{
				turn16(true, serial);//vasakule
			}
		}
		//change position
	}
}

void aim_ball(float kaugus, Point2f mc, SerialClass& serial){

	stop_dribbler(serial);
	int direction;
	if (mc.x == -1){
		
	}
	else if (kaugus > 100){//kaugel
		if (mc.x < 320){
			direction = 1;
			turn(30, direction, serial);
		}
		else if (mc.x > 320){
			direction = 0;
			turn(30, direction, serial);
		}
		else{
			
		}
	}
	else{//lähedal
		if (mc.x < 320){
			direction = 1;
			turn(30, direction, serial);
		}
		else if (mc.x > 320){
			direction = 0;
			turn(30, direction, serial);
		}
		else{
			
		}
	}
}

void find_ball(Mat frame, vector<int> ball, SerialClass& serial){
	stop_dribbler(serial);

	
	Point2f mc;
	float kaugus;

	while (mc.x == -1){
		for (int i = 0; i < 6; i++){//max 6 times
			tie(frame, mc, kaugus) = get_frame_ball(frame, ball);
			if (mc.x != -1){
				break;
			}
			else{
				turn16(true, serial);//vasakule
			}
		}
		
		//change position
	}
}

void change_position_ball(Mat frame){
	Point2f p1, p2;
	tie(frame, p1, p2) = get_frame_line(frame);
	if (p1.x != -1){//joon näha
		int nurk = tous(p1, p2);
		cout << nurk << endl;
	}
}

int tous(Point2f p1, Point2f p2){
	float param = (p2.y - p1.y) / (p2.x - p1.x);
	int nurk = atan(param) * 180 / CV_PI;
	return nurk;
}

void catch_ball(Mat frame){

}

void otse(int speed, SerialClass& serial){
	float liigu[3] = { 1, 0, 0 };
	movement(liigu, speed, serial);
}

tuple<Mat, Point2f, Point2f> get_frame_line(Mat frame){
	
	Mat black_thresh, black_result, white_thresh, white_result;
	
	black_thresh = preprocess(frame, 0, 180, 0, 255, 0, 160);
	white_thresh = preprocess(frame, 0, 180, 0, 255, 160, 255);

	dilate(white_thresh, white_thresh, Mat(), Point(-1, -1), 10);
	bitwise_and(white_thresh, black_thresh, white_result);

	findContours(white_result, contours_white, hierarchy_white, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	Point2f mc_black, mc_white, corner1, corner2;

	tie(mc_white, corner1, corner2) = process_goal(contours_white, frame, Scalar(255, 0, 0));
	
	return make_tuple(frame, corner1, corner2);
	
}

//check if goal in view
tuple<Mat, Point2f> get_frame_goal(Mat frame, vector<int> goal){
	Mat goal_thresh;
	
	goal_thresh = preprocess(frame, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
	findContours(goal_thresh, contours_goal1, hierarchy_goal, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Point2f mc_goal, corner0, corner1;

	tie(mc_goal, corner0, corner1) = process_goal(contours_goal1, frame, Scalar(255, 255, 255));
	return make_tuple(frame, mc_goal);
}

//check if ball in view
tuple<Mat, Point2f, float> get_frame_ball(Mat frame, vector<int> ball){
	Mat ball_thresh;

	ball_thresh = preprocess(frame, ball[0], ball[1], ball[2], ball[3], ball[4], ball[5]);
	findContours(ball_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Point2f mc_ball;
	float kaugus;
	tie(mc_ball, kaugus) = process_ball(contours_ball, frame);
	return make_tuple(frame, mc_ball, kaugus);
}

int main() {
	int state = 1;//select state 0-calib color, no serial; 1- competition mode, no trackbars
	Mat frame;
	Point2f mc_ball, mc_goal;
	float kaugus;
	int speed = 150;

	ifstream calib_param;
	calib_param.open("C:\\Users\\Sarvik\\Documents\\GitHub\\Robootika_praks\\calib_param.txt");
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
	/*
	ball_calib[0] = 5;
	ball_calib[1] = 25;
	ball_calib[2] = 80;
	ball_calib[3] = 255;
	ball_calib[4] = 50;
	ball_calib[5] = 255;
	yellow_calib[0] = 15;
	yellow_calib[1] = 40;
	yellow_calib[2] = 100;
	yellow_calib[3] = 255;
	yellow_calib[4] = 70;
	yellow_calib[5] = 255;
	blue_calib[0] = 90;
	blue_calib[1] = 108;
	blue_calib[2] = 160;
	blue_calib[3] = 255;
	blue_calib[4] = 70;
	blue_calib[5] = 221;
	*/
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
			Mat frame;
			cap >> frame;
			if (!cap.read(frame)) std::cout << "error reading frame" << endl;//check for error'
			Point2f corner1;
			float kaugus;
			
			tie(frame, corner1, kaugus) = get_frame_ball(frame, ball_calib);
			//cout << kaugus << endl;
			//tie(frame, mc_ball, mc_goal, kaugus) = get_frame(cap, ball_calib, yellow_calib, blue_calib, state);
			imshow("calibrate", frame);
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
			
			cap >> frame;
			if (!cap.read(frame)) std::cout << "error reading frame" << endl;//check for error'
			
			/*
			while (stopbool ==true){
			Mat frame;
			Point2f corner1, corner2;
			float kaugus;
			tie(frame, corner1, kaugus) = get_frame_ball(cap, ball_calib);
			cout << kaugus << endl;
			imshow("test", frame);
			waitKey(10);
			}
			*/
			if (respond == true){
				String temp = "a" + my_field + my_robotID + "ACK-----\r";
				respond = false;
			}

			serial.send("bl\r\n");

			if (bl == true){
				Point2f g;
				if (goal_select == true){
					tie(frame, g) = get_frame_goal(frame, yellow_calib);
				}
				else{
					tie(frame, g) = get_frame_goal(frame, blue_calib);
				}
				if (g.x == -1){
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
				Point2f b;
				float kaugus;
				tie(frame, b, kaugus) = get_frame_ball(frame, ball_calib);
				if (b.x == -1){
					
					//search ball
					/*
					turn max 6 times(360deg), if no ball found move on
					*/
					for (int i = 0; i < 6; i++){
						cap >> frame;
						Point2f p1, p2;
						tie(frame, p1, p2) = get_frame_line(frame);
						if (p1.x != -1){//joon on näha
							int nurk = tous(p1, p2);
							//cout << nurk << endl;

						}
						else{//joont pole näha
							turn16(true, serial);
						}
						imshow("orig", frame);
						waitKey(10);
					}
					cout << "turned" << endl;
					//sleepcp(500);

				}
				else{
					//handle catching
					/*
					turn to ball
					if path clear, move to ball
					else find better position
					*/
					
					Point2f p1, p2;
					tie(frame, p1, p2) = get_frame_line(frame);
					if (p1.x != -1){//joon on näha
						int nurk = tous(p1, p2);
				
						if (p1.y>p2.y){
							int pallinurk = tous(b, p1);
							if (nurk < 0){
								if (pallinurk < nurk){
									cout << "väljas" << endl;
									//cout << "sees" << endl;
									//sleepcp(2000);
								}
								else{
									cout << "sees" << endl;

									if ((b.x > 275) && (b.x < 365)){
										//OTSE
										//cout << "otse" << endl;
										otse(75, serial);
										
									}
									else{
										aim_ball(kaugus, b, serial);
									}
								}
							}
							else{
								if (pallinurk > nurk){
									cout << "väljas" << endl;
									//sleepcp(2000);
								}
								else{
									cout << "sees" << endl;

									if ((b.x > 275) && (b.x < 365)){
										//OTSE
										//cout << "otse" << endl;
										otse(75, serial);

									}
									else{
										aim_ball(kaugus, b, serial);
									}
								}
							}
						}
						else{
							int pallinurk = tous(b, p2);
							if (nurk < 0){
								if (pallinurk < nurk){
									cout << "väljas" << endl;
									//sleepcp(2000);
								}
								else{
									cout << "sees" << endl;

									if ((b.x > 275) && (b.x < 365)){
										//OTSE
										//cout << "otse" << endl;
										otse(75, serial);

									}
									else{
										aim_ball(kaugus, b, serial);
									}
								}
							}
							else{
								if (pallinurk > nurk){
									cout << "väljas" << endl;
									//sleepcp(2000);
								}
								else{
									cout << "sees" << endl;

									if ((b.x > 275) && (b.x < 365)){
										//OTSE
										//cout << "otse" << endl;
										otse(75, serial);

									}
									else{
										aim_ball(kaugus, b, serial);
									}
								}
							}
						}

					}
					else{//joont pole näha
						if ((b.x > 275) && (b.x < 365)){
							//OTSE
							//cout << "otse" << endl;
							otse(75, serial);

							
						}
						else{
							aim_ball(kaugus, b, serial);
						}
					}
				}
			}

			imshow("orig", frame);
			waitKey(10);
		
		}
	}

	return 0;
}