#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

using namespace std;
using namespace cv;
#define DISTANCE_SLICE_TO_CHANGE_STEER 25
vector<Point> unWarpPoint(vector<Point>& pnt, Mat& warpMatrixInv);
class DetectLane;
class Obstacle{
    private:
        vector<Rect> obstacle_coordinate;
        vector<vector<Point>> resultPoint;
    public:
        void set(vector<Rect>& coordinate){
            obstacle_coordinate = coordinate;
        }
        void clear(){
            obstacle_coordinate.clear();
        }
        bool is_there_obstacle(){
            return obstacle_coordinate.size();
        }
        void calc_warp(Mat& warpMatrix);
        //
        Point offsetAngle(Point& carPos, Point &Steer, float car_width, float car_height, DetectLane& detect, bool is_steer_middle);
};
class DetectObstacle {
    public:
        void init(const string& rock_dir, const string& stacking_dir);//, const string& right_haar_dir);
        Obstacle get_obstacle(bool draw=false);
        void store_image(Mat& image);
        Mat& get_image();
    private:
        int init_flag = 0;
        Mat* image;
        //Obstacle obstacle;
        CascadeClassifier rock_detect;
        CascadeClassifier stacking_detect;
        bool check_obstacle(Obstacle& coordinate, bool draw=false);
        //static CascadeClassifier right_traffic_sign_detect;
        //static bool check_traffic_sign_left(bool draw=false);
};