#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv\cv.h>

using namespace cv;
using namespace std;

float eucl_dist(Point2f corner1, Point2f corner2);
Mat preprocess(Mat frame, int lowH, int lowS, int lowV, int highH, int highS, int highV);
tuple<Point2f, Point2f, Point2f> process_goal(vector<vector<Point>> contours, Mat frame, Scalar varv);
tuple<Point2f, float> process_ball(vector<vector<Point>> contours, Mat frame);