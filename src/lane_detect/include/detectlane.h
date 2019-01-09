#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;
Scalar ConvertArraytoScalar(int arr[3]);
float angleVector(const Vec2f& vec1, const Vec2f& vec2);
float angleLine(const Vec4f& line1, const Vec4f& line2);
Vec4f createLine(const Point& pnt1, const Point& pnt2);
float dist_point_point(const Point& pnt1, const Point& pnt2);
float dist_point_line(const Point& pnt, const Vec4f& line);
float signed_dist_point_line(const Point& pnt, const Vec4f& line);
Point intersection (const Vec4f& line1, Vec4f& line2);
float calc_X(const int& y, const Vec4f& line);
float calc_Y(const int& x, const Vec4f& line);
bool check_segment_list(vector<vector<Point>>& segment_list);
//class CarControl;
#define SKYLINE 85
//#define SKYLINE 85
// For birdViewTranform
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define BIRDVIEW_WIDTH 240
#define BIRDVIEW_HEIGHT 320
#define VERTICAL 0
#define HORIZONTAL 1
#define slideThickness 10
extern Point2f src_vertices[4];
extern Point2f dst_vertices[4];
enum SIDE {
    LEFT,
    MIDDLE,
    RIGHT,
    UNDEFINED
    //NONE,
    //ERR
};
class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    vector<Point> getLeftLaneRaw(){
        return leftLaneRaw;
    };
    vector<Point> getRightLaneRaw(){
        return rightLaneRaw;
    }
    vector<Point> getMiddleLaneRaw(){
        return middleLaneRaw;
    }
    SIDE getMiddleLaneSide(){
        return middleLaneSide;
    }
    //int getLeftLaneSize(){
    //    return leftLane_size;
    //}
    //int getRightLaneSize(){
    //    return rightLane_size;
    //}

    //static int slideThickness;

    //static int BIRDVIEW_WIDTH;
    //static int BIRDVIEW_HEIGHT;

    //static int VERTICAL;
    //static int HORIZONTAL;

    static Point null; //

    Vec4f vecLeft, vecRight;  

    //enum MIDDLE
private:
    Mat preProcess(const Mat &src);
    void preProcess(const Mat &src, Mat& imgRoad, Mat& imgLane);
    void preProcess(const Mat &src, Mat& imgLane);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    //void detectLeftRight(const vector<vector<Point> > &points);
    void detectLeftRight(const vector<vector<Point> > &points, vector<Point>& leftRaw, vector<Point>& rightRaw);
    void detectLane(const vector<vector<Point> > &points, vector<Point>& leftRaw, vector<Point>& rightRaw, vector<Point>& middleRaw);
    void updateLeftRight(const vector<Point>& leftRaw, const vector<Point>& rightRaw, vector<Point>& left, vector<Point>& right);
    void updateLane(const vector<Point>& leftRaw, const vector<Point>& rightRaw, const vector<Point>& middleRaw, vector<Point>& left, vector<Point>& right, vector<Point>& middle);
    Mat laneInShadow(const Mat &src);
    /*
    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};*/
    //int minThreshold[3] = {101, 68, 0};
    //int maxThreshold[3] = {126, 196, 255};
    
    int minLaneShadowTh[3] = {66, 72, 24};
    int maxLaneShadowTh[3] = {111, 104, 58};
    int minLaneNormalTh[3] = {66, 111, 0};
    int maxLaneNormalTh[3] = {111, 255, 255};
    //int minLaneNormalTh[3] = {55, 155, 0};
    //int maxLaneNormalTh[3] = {179, 255, 184};
    //int minThreshold[3] = {55, 93, 0};

    int minRoadNormalTh[3] = {34, 0, 0};
    int maxRoadNormalTh[3] = {96, 93, 14};
    int minRoadShadowTh[3] = {83, 0, 15};
    int maxRoadShadowTh[3] = {179, 127, 67};
    int binaryThreshold = 180;

    int shadowParam = 40;

    SIDE middleLaneSide;
    vector<Point> leftLane, rightLane, middleLane;
    vector<Point> leftBound, rightBound;
    vector<Point> leftLaneRaw, rightLaneRaw, middleLaneRaw;
    //vector<Point> leftBoundRaw, rightBoundRaw;
};

#endif
