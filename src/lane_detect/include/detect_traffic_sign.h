//#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

enum TRAFFIC_SIGN {
    TURN_LEFT,
    TURN_RIGHT,
    NONE
};
class DetectSign {
    public:
        static void init(const string& left_haar_dir, const string& right_haar_dir);
        static TRAFFIC_SIGN get_traffic_sign(bool draw=false);
        static void store_image(Mat& image);
        static Mat& get_image();
    private:
        static Mat* image;
        static CascadeClassifier left_traffic_sign_detect;
        static CascadeClassifier right_traffic_sign_detect;
        static bool check_traffic_sign_left(bool draw=false);
        static bool check_traffic_sign_right(bool draw=false);
};
