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

    
private:
    Mat preProcess(const Mat &src);
    Mat preProcess(const Mat &src, Mat& imgRoad, Mat& imgLane);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    //void detectLeftRight(const vector<vector<Point> > &points);
    void detectLeftRight(const vector<vector<Point> > &points, vector<Point>& leftRaw, vector<Point>& rightRaw);
    void updateLeftRight(const vector<Point>& leftRaw, const vector<Point>& rightRaw, vector<Point>& left, vector<Point>& right);
    Mat laneInShadow(const Mat &src);
    /*
    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};*/
    //int minThreshold[3] = {101, 68, 0};
    //int maxThreshold[3] = {126, 196, 255};
    
    int minLaneShadowTh[3] = {55, 155, 0};
    int maxLaneShadowTh[3] = {179, 255, 184};
    int minLaneNormalTh[3] = {55, 155, 0};
    //int minThreshold[3] = {55, 93, 0};
    int maxLaneNormalTh[3] = {179, 255, 184};
    int minRoadShadowTh[3] = {90, 43, 36};
    int maxRoadShadowTh[3] = {120, 81, 171};
    int minRoadNormalTh[3] = {55, 155, 0};
    int maxRoadNormalTh[3] = {179, 255, 184};
    int binaryThreshold = 180;

    int shadowParam = 40;

    vector<Point> leftLane, rightLane; //, middleLane;
    vector<Point> leftBound, rightBound;
    vector<Point> leftLaneRaw, rightLaneRaw;
    //vector<Point> leftBoundRaw, rightBoundRaw;
};

#endif
