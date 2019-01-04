#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;

vector <float> fitPoly(const vector <Point2f> &src, int order);
void cvPolyfit(cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order);