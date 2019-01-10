#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

class Obstacle{
    public:
        vector<Rect> obstacle_coordinate;
        vector<Point> resultPoint;
};
class DetectObstacle {
    public:
        void init(const string& rock_dir);//, const string& right_haar_dir);
       static Obstacle get_rock(bool draw=false);
        void store_image(Mat& image);
        Mat& get_image();
    private:
        int init_flag;
        Mat* image;
        CascadeClassifier rock_detect;
        //static CascadeClassifier right_traffic_sign_detect;
        //static bool check_traffic_sign_left(bool draw=false);
        bool check_rock(bool draw=false);
};