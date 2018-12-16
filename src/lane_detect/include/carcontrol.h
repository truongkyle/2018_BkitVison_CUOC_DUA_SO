#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

class DetectLane;

class CarControl 
{
public:
    CarControl();
    ~CarControl();
    void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity);
    void driverCar(DetectLane* detect);
    void driverCar(DetectLane* detect, const Mat& src);
    static void init();
    static Mat& getROI(){
        return *maskROI;
    }
private:
    float errorAngle(const Point &dst);
    float errorAngle(const Point &, const Point &);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = 40;

    float minVelocity = 10;
    float maxVelocity = 50;

    float preError;

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;

    static Mat* maskROI;
    static Mat maskRoiLane;
    static Mat maskRoiIntersection;
    static Point LeftAboveSpeedA, RightAboveSpeedA, LeftBelowSpeedA, RightBelowSpeedA;
    static Point LeftAboveAngleA, RightAboveAngleA, LeftBelowAngleA, RightBelowAngleA;
    static Point preSteer;
    static float preSpeed;
    static Vec2f vecLeftSpeed, vecRightSpeed, vecLeftAngle, vecRightAngle;
};
//init();
#endif
