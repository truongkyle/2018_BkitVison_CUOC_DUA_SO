#include "carcontrol.h"
#include "detectlane.h"
#include "my_define.h"
#define MAX_CONTINUITY_DIST 3
#define MIN_DIST_SEGMENT 13
#define UPPER_MAX_SEGMENT_SIZE 8
#define LOWER_MAX_SEGMENT_SIZE 5
#define SEGMENT_HEIGHT_LIMIT BIRDVIEW_HEIGHT/2
#define CUTOFF_HEIGHT_LANE_DETECT 0
#define MIN_SEGMENT_NUMBER_REQUIREMENT 3
#define SEGMENT_SIZE_REQUIREMENT 3
#define DIST_SIDE_LANE_MID_LANE_UPPER_LIM 35
#define DIST_SIDE_LANE_MID_LANE_LOWER_LIM 25

int min(int a, int b){
    return a < b ? a : b;
}
int maxx(int a, int b){
    return a > b ? a : b;
}
Scalar ConvertArraytoScalar(int arr[3]);
float angleVector(const Vec2f& vec1, const Vec2f& vec2);
float angleLine(const Vec4f& line1, const Vec4f& line2);
Vec4f createLine(const Point& pnt1, const Point& pnt2);
float dist_point_point(const Point& pnt1, const Point& pnt2);
float signed_dist_point_line(const Point& pnt, const Vec4f& line);
float dist_point_line(const Point& pnt, const Vec4f& line);
Point intersection (const Vec4f& line1, Vec4f& line2);
float calc_X(const int& y, const Vec4f& line);
float calc_Y(const int& x, const Vec4f& line);
bool check_segment_list(vector<vector<Point>>& segment_list);
//int DetectLane::slideThickness = 10;
/*
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;
*/
Point2f src_vertices[4] = {
    Point(0, SKYLINE),
    Point(IMAGE_WIDTH, SKYLINE),
    Point(IMAGE_WIDTH, IMAGE_HEIGHT),
    Point(0, IMAGE_HEIGHT)
};

Point2f dst_vertices[4] = {
    Point(0, 0),
    Point(BIRDVIEW_WIDTH, 0),
    Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT),
    Point(105, BIRDVIEW_HEIGHT)
};
Point DetectLane::null = Point();

DetectLane::DetectLane() {
    cvCreateTrackbar("Low-H-roadNormal", "ThresholdRoad", &minRoadNormalTh[0], 179);
    cvCreateTrackbar("High-H-roadNormal", "ThresholdRoad", &maxRoadNormalTh[0], 179);

    cvCreateTrackbar("Low-S-roadNormal", "ThresholdRoad", &minRoadNormalTh[1], 255);
    cvCreateTrackbar("High-S-roadNormal", "ThresholdRoad", &maxRoadNormalTh[1], 255);

    cvCreateTrackbar("Low-V-roadNormal", "ThresholdRoad", &minRoadNormalTh[2], 255);
    cvCreateTrackbar("High-V-roadNormal", "ThresholdRoad", &maxRoadNormalTh[2], 255);

    cvCreateTrackbar("Low-H-roadShadow", "ThresholdRoad", &minRoadShadowTh[0], 179);
    cvCreateTrackbar("High-H-roadShadow", "ThresholdRoad", &maxRoadShadowTh[0], 179);

    cvCreateTrackbar("Low-S-roadShadow", "ThresholdRoad", &minRoadShadowTh[1], 255);
    cvCreateTrackbar("High-S-roadShadow", "ThresholdRoad", &maxRoadShadowTh[1], 255);

    cvCreateTrackbar("Low-V-roadShadow", "ThresholdRoad", &minRoadShadowTh[2], 255);
    cvCreateTrackbar("High-V-roadShadow", "ThresholdRoad", &maxRoadShadowTh[2], 255);

    //cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);

    cvCreateTrackbar("Low-H-laneNormal", "ThresholdLane", &minLaneNormalTh[0], 179);
    cvCreateTrackbar("High-H-laneNormal", "ThresholdLane", &maxLaneNormalTh[0], 179);

    cvCreateTrackbar("Low-L-laneNormal", "ThresholdLane", &minLaneNormalTh[1], 255);
    cvCreateTrackbar("High-L-laneNormal", "ThresholdLane", &maxLaneNormalTh[1], 255);

    cvCreateTrackbar("Low-S-laneNormal", "ThresholdLane", &minLaneNormalTh[2], 255);
    cvCreateTrackbar("High-S-laneNormal", "ThresholdLane", &maxLaneNormalTh[2], 255);

    cvCreateTrackbar("Low-H-laneShadow", "ThresholdLane", &minLaneShadowTh[0], 179);
    cvCreateTrackbar("High-H-laneShadow", "ThresholdLane", &maxLaneShadowTh[0], 179);

    cvCreateTrackbar("Low-L-laneShadow", "ThresholdLane", &minLaneShadowTh[1], 255);
    cvCreateTrackbar("High-L-laneShadow", "ThresholdLane", &maxLaneShadowTh[1], 255);

    cvCreateTrackbar("Low-S-laneShadow", "ThresholdLane", &minLaneShadowTh[2], 255);
    cvCreateTrackbar("High-S-laneShadow", "ThresholdLane", &maxLaneShadowTh[2], 255);
}

DetectLane::~DetectLane(){}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane(){
    return rightLane;
}
int countFrame = 0;
void DetectLane::update(const Mat &src){
    //cout << countFrame++ << endl;
    //cout << "hihi" << endl;
    //Mat edges;
    //Canny(src, edges, 100, 200);
    //cv::imshow("Canny", edges);
    Mat imgLane, imgRoad;
    #if DETECT_METHOD == WHITE_LANE
        preProcess(src, imgLane);
        vector<Mat> layers1 = splitLayer(imgLane);
    #elif DETECT_METHOD == ROAD_COLOR
        imgRoad = preProcess(src);
        vector<Mat> layers1 = splitLayer(imgRoad);
    #endif
    vector<vector<Point> > points1 = centerRoadSide(layers1);
    // vector<Mat> layers2 = splitLayer(img, HORIZONTAL);
    // vector<vector<Point> > points2 = centerRoadSide(layers2, HORIZONTAL)
    //cout << points1.size() << endl;
    //Mat temp3d_1 = Mat::zeros(Size(240,320), CV_8UC3);
    //for (int i = 0; i < points1.size(); i++){
    //    int slice_thickness;
    //    //cout << points1[i].size() << endl;
    //    for (int j = 0; j < points1[i].size(); j++)
    //        cv::circle(temp3d_1, points1[i][j], 1, Scalar(255,0,0), 2);
    //};
    //cout << "hehe" << endl;
    //cv::circle(temp3d_1, Point(50, 60), 1, Scalar(255,0,0), 2);
    //cv::imshow("Point", temp3d_1);
    //detectLeftRight(points1);
    //detectLeftRight(points1, leftLaneRaw, rightLaneRaw);
    #if DETECT_METHOD == WHITE_LANE
        detectLane(points1, leftLaneRaw, rightLaneRaw, middleLaneRaw);
        updateLane(leftLaneRaw, rightLaneRaw, middleLaneRaw, leftLane, rightLane, middleLane);
    #elif DETECT_METHOD == ROAD_COLOR
        detectLeftRight(points1, leftLaneRaw, rightLaneRaw);
        updateLeftRight(leftLaneRaw, rightLaneRaw, leftLane, rightLane);
    #endif
    Mat birdView, lane;
    //birdView = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    lane = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    //for (int i = 0; i < points1.size(); i++)
    //{
    //    for (int j = 0; j < points1[i].size(); j++)
    //    {
    //        circle(birdView, points1[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
    //    }
    //}

    /*
    vector<Vec2f> lines;
    HoughLines(birdView, lines, 1, CV_PI/180, 50);
    float pos_theta = 0,
          pos_rho = 0,
          pos_cnt = 0,
          neg_theta = 0,
          neg_rho = 0,
          neg_cnt = 0;
    for (vector<Vec2f>::iterator it = lines.begin(); it != lines.end(); ++it){
        float rho = (*it)[0];
        float theta = (*it)[1];
        if (theta >= CV_PI/2){
            pos_theta += theta;
            pos_rho += rho;
            pos_cnt++;
        }
        if (theta <= CV_PI/2){
            neg_theta += theta;
            neg_rho += rho;
            neg_cnt++;
        }
    }
    if (pos_cnt > 0){
        pos_rho /= pos_cnt;
        pos_theta /= pos_cnt;
    }
    if (neg_cnt > 0){
        neg_rho /= neg_cnt;
        neg_theta /= neg_cnt;
    }
    float a_pos = cos(pos_theta),
          b_pos = sin(pos_theta),
          a_neg = cos(neg_theta),
          b_neg = sin(neg_theta),
          x0_pos = a_pos * pos_rho,
          y0_pos = b_pos * pos_rho,
          x0_neg = a_neg * neg_rho,
          y0_neg = b_neg * neg_rho;
    Point pt1_pos(int(x0_pos + 1000*(-b_pos)), int(y0_pos + 1000*(a_pos))),
          pt2_pos(int(x0_pos - 1000*(-b_pos)), int(y0_pos - 1000*(a_pos))),
          pt1_neg(int(x0_neg + 1000*(-b_neg)), int(y0_neg + 1000*(a_neg))),
          pt2_neg(int(x0_neg - 1000*(-b_neg)), int(y0_neg - 1000*(a_neg)));

    cv::line(birdView, pt1_pos, pt2_pos, 1, 3, CV_AA);
    cv::line(birdView, pt1_neg, pt2_neg, 1, 3, CV_AA);
    cv::imshow("HoughLines", birdView);*/
    // for (int i = 0; i < points2.size(); i++)
    //  {
    //     for (int j = 0; j < points2[i].size(); j++)
    //     {
    //         circle(birdView, points2[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
    //     }
    // }

    // imshow("Debug", birdView);

    for (int i = 1; i < leftLane.size(); i++){
        if (leftLane[i] != null) {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++){
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }

    for (int i = 1; i < middleLane.size(); i++){
        if (middleLane[i] != null) {
            circle(lane, middleLane[i], 1, Scalar(0,255,0), 2, 8, 0 );
        }
    }

    cv::imshow("Lane Detect", lane);
}

Mat DetectLane::preProcess(const Mat &src){
    /*
    Mat imgThresholded, imgHLS, dst;

    cvtColor(src, imgHLS, COLOR_BGR2HLS);
    
    cv::GaussianBlur(imgHLS, imgHLS, Size(17,17), 0.);
//cout << "test";

    //cout << maskROI3d.size();
    //cout << blur_img.size();
    //Mat maskROI;
    cv::bitwise_and(CarControl::getROI(), imgHLS, imgHLS);
//cout << "test";
    Mat mask1;
    cv::inRange(imgHLS, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
                Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), mask1);
    //Mat mask2;
    //cv::inRange(imgHSV, Scalar(20,56,43), Scalar(35,77,64), mask2);
    //Mat maskShadow;
    //cv::inRange(imgHSV, Scalar(70,28,12), Scalar(137,145,74), maskShadow);
    //mask side lane

    //Mat maskCombined = mask1 | mask2 | maskShadow;
    Mat& maskCombined = mask1;
    cv::imshow("Threshold", maskCombined);
    vector<vector<Point>> contours;
    cv::findContours(maskCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //cout << "test";
    vector<Point> max_contour;
    double max_area = 0;
    for (const vector<Point>& contour : contours){
        double cnt_area = cv::contourArea(contour);
        if (cnt_area > max_area){
            max_area = cnt_area;
            max_contour = contour;
        }
    }
    //vector<vector<Point>> max_contour_dummy;
    //max_contour_dummy.push_back(max_contour);
    maskCombined = Mat::zeros(Size(320,240), CV_8U);
    cv::drawContours(maskCombined, vector<vector<Point>>{max_contour}, -1, 255, 2);  
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    cv::imshow("Binary", maskCombined);
    
    // for (int i = 0; i < points2.size(); i++)
    dst = birdViewTranform(maskCombined);
    fillLane(dst);
    cv::imshow("Bird View", dst);
    return dst;
    */

   /*
    Mat imgThresholded, imgHLS, dst;

    cvtColor(src, imgHLS, COLOR_BGR2HLS);
    
    cv::GaussianBlur(imgHLS, imgHLS, Size(17,17), 0.);
//cout << "test";

    //cout << maskROI3d.size();
    //cout << blur_img.size();
    //Mat maskROI;
    cv::bitwise_and(CarControl::getROI(), imgHLS, imgHLS);
//cout << "test";
    Mat mask1;
    cv::inRange(imgHLS, Scalar(102,43,10), Scalar(123,212,105), mask1);
    //mask side lane

    Mat maskCombined = mask1;

    vector<vector<Point>> contours;
    cv::findContours(maskCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
\\\\\\\\\\//cout << "test";
    vector<Point> max_contour;
    double max_area = 0;
    for (vector<Point> contour : contours){
        double cnt_area = cv::contourArea(contour);
        if (cnt_area > max_area){
            max_area = cnt_area;
            max_contour = contour;
        }
    }
    vector<vector<Point>> max_contour_dummy;
    max_contour_dummy.push_back(max_contour);
    maskCombined = Mat::zeros(Size(320,240), CV_8U);
    cv::drawContours(maskCombined, max_contour_dummy, -1, 255, 2);  
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    cv::imshow("Binary", maskCombined);
    // for (int i = 0; i < points2.size(); i++)
    dst = birdViewTranform(maskCombined);
    fillLane(dst);
    cv::imshow("Bird View", dst);
    return dst;
    */

    /*
    Mat imgThresholded, imgHLS, dst, blur_img;

    blur_img = src & CarControl::getROI();
    
    cv::GaussianBlur(blur_img, blur_img, Size(7,7), 0.);
    
    cvtColor(blur_img, imgHLS, COLOR_BGR2HLS);
    
    inRange(imgHLS, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), 
        imgThresholded);

    // Or with shadhow threshold

    dst = birdViewTranform(imgThresholded);

    fillLane(dst); oh. e chưa đổi tên source path trong cmakelist

    dst = morphological(dst);

    imshow("Bird View", dst);

    imshow("Threshold", imgThresholded);

    return dst;
    */
    
    Mat imgThresholded, imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);
    
    cv::GaussianBlur(imgHSV, imgHSV, Size(17,17), 0.);
//cout << "test";

    //cout << maskROI3d.size();
    //cout << blur_img.size();
    //Mat maskROI;
    cv::bitwise_and(CarControl::getROI(), imgHSV, imgHSV);
//cout << "test";
    Mat mask1;
    cv::inRange(imgHSV, ConvertArraytoScalar(minRoadNormalTh), ConvertArraytoScalar(maxRoadNormalTh), mask1);
    //Mat mask2;
    //cv::inRange(imgHSV, Scalar(20,56,43), Scalar(35,77,64), mask2);
    Mat maskShadow;
    cv::inRange(imgHSV, ConvertArraytoScalar(minRoadShadowTh), ConvertArraytoScalar(maxRoadShadowTh), maskShadow);
    Mat maskCombined;
    maskCombined = mask1 | maskShadow;

    vector<vector<Point>> contours;
    cv::findContours(maskCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //cout << contours.size() << endl;
    //cout << "test";                CarControl::preSteer = craftPoint(carPos, vecRightSpeed);
    //vector<Point> max_contour;
    int max_contour = -1;
    double max_area = 0;
    for (int i = 0; i < contours.size(); i++){
        double cnt_area = cv::contourArea(contours[i]);
        if (cnt_area > max_area){
            max_area = cnt_area;
            max_contour = i;
        }
    }
    //vector<vector<Point>> max_contour_dummy;
    //max_contour_dummy.push_back(max_contour);
    //cout << max_contour_dummy.size() << endl;
    maskCombined = Mat::zeros(Size(320,240), CV_8U);
    //cout << max_contour << endl;
    if (max_contour >= 0) cv::drawContours(maskCombined, vector<vector<Point>>{contours[max_contour]}, -1, Scalar(255), 2);  
    //cout << contours.size() << endl;
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    imshow("ThresholdRoad", maskCombined);
    // for (int i = 0; i < points2.size(); i++)
    dst = birdViewTranform(maskCombined);
    fillLane(dst);
    imshow("Bird View Lane", dst);
    return dst;
    
}
//void DetectLane::preProcess(const Mat &src, Mat& imgRoad, Mat& imgLane);
void DetectLane::preProcess(const Mat &src, Mat& imgLane){
    /*
    Mat imgThresholded, imgHLS, dst;

    cvtColor(src, imgHLS, COLOR_BGR2HLS);
    
    cv::GaussianBlur(imgHLS, imgHLS, Size(17,17), 0.);
//cout << "test";

    //cout << maskROI3d.size();
    //cout << blur_img.size();
    //Mat maskROI;
    cv::bitwise_and(CarControl::getROI(), imgHLS, imgHLS);
//cout << "test";
    Mat mask1;
    cv::inRange(imgHLS, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
                Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), mask1);
    //Mat mask2;
    //cv::inRange(imgHSV, Scalar(20,56,43), Scalar(35,77,64), mask2);
    //Mat maskShadow;
    //cv::inRange(imgHSV, Scalar(70,28,12), Scalar(137,145,74), maskShadow);
    //mask side lane

    //Mat maskCombined = mask1 | mask2 | maskShadow;
    Mat& maskCombined = mask1;
    cv::imshow("Threshold", maskCombined);
    vector<vector<Point>> contours;
    cv::findContours(maskCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //cout << "test";
    vector<Point> max_contour;
    double max_area = 0;
    for (const vector<Point>& contour : contours){
        double cnt_area = cv::contourArea(contour);
        if (cnt_area > max_area){
            max_area = cnt_area;
            max_contour = contour;
        }
    }
    //vector<vector<Point>> max_contour_dummy;
    //max_contour_dummy.push_back(max_contour);
    maskCombined = Mat::zeros(Size(320,240), CV_8U);
    cv::drawContours(maskCombined, vector<vector<Point>>{max_contour}, -1, 255, 2);  
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    cv::imshow("Binary", maskCombined);
    
    // for (int i = 0; i < points2.size(); i++)
    dst = birdViewTranform(maskCombined);
    fillLane(dst);
    cv::imshow("Bird View", dst);
    return dst;
    */

   /*
    Mat imgThresholded, imgHLS, dst;

    cvtColor(src, imgHLS, COLOR_BGR2HLS);
    
    cv::GaussianBlur(imgHLS, imgHLS, Size(17,17), 0.);
//cout << "test";

    //cout << maskROI3d.size();
    //cout << blur_img.size();
    //Mat maskROI;
    cv::bitwise_and(CarControl::getROI(), imgHLS, imgHLS);
//cout << "test";
    Mat mask1;
    cv::inRange(imgHLS, Scalar(102,43,10), Scalar(123,212,105), mask1);
    //mask side lane

    Mat maskCombined = mask1;

    vector<vector<Point>> contours;
    cv::findContours(maskCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //cout << "test";
    vector<Point> max_contour;
    double max_area = 0;
    for (vector<Point> contour : contours){
        double cnt_area = cv::contourArea(contour);
        if (cnt_area > max_area){
            max_area = cnt_area;
            max_contour = contour;
        }
    }
    vector<vector<Point>> max_contour_dummy;
    max_contour_dummy.push_back(max_contour);
    maskCombined = Mat::zeros(Size(320,240), CV_8U);
    cv::drawContours(maskCombined, max_contour_dummy, -1, 255, 2);  
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    cv::imshow("Binary", maskCombined);
    // for (int i = 0; i < points2.size(); i++)
    dst = birdViewTranform(maskCombined);
    fillLane(dst);
    cv::imshow("Bird View", dst);
    return dst;
    */

    /*
    Mat imgThresholded, imgHLS, dst, blur_img;

    blur_img = src & CarControl::getROI();
    
    cv::GaussianBlur(blur_img, blur_img, Size(7,7), 0.);
    
    cvtColor(blur_img, imgHLS, COLOR_BGR2HLS);
    
    inRange(imgHLS, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), 
        imgThresholded);

    // Or with shadhow threshold

    dst = birdViewTranform(imgThresholded);

    fillLane(dst);

    dst = morphological(dst);

    imshow("Bird View", dst);

    imshow("Threshold", imgThresholded);

    return dst;
    */
    
    Mat imgThresholded, imgHLS, blur_imgHLS;

    //cv::imshow("ThresholdLane", CarControl::getROI());
    cvtColor(src, imgHLS, COLOR_BGR2HLS);

    //cv::GaussianBlur(imgHLS, blur_imgHLS, Size(17,17), 0.);//, 0.);
    //cv::medianBlur(src, blur_image, 11);
    
    //imgHLS = imgHLS & CarControl::getROI();
    //blur_imgHLS = blur_imgHLS & CarControl::getROI();
    //cvtColor(blur_imgHLS, blur_imgHLS, COLOR_BGR2HLS);
    
//cout << "test";

    //cout << maskROI3d.size();
    //cout << blur_img.size();
    //Mat maskROI;
    //imgHLS = imgHLS & CarControl::getROI();
    //blur_imgHLS = blur_imgHLS & CarControl::getROI();
//cout << "test";
    //Mat maskRoadNormal;
    //cv::inRange(blur_imgHLS, ConvertArraytoScalar(minRoadNormalTh), ConvertArraytoScalar(maxRoadNormalTh), maskRoadNormal);
    //Mat mask2;
    //cv::inRange(imgHSV, Scalar(20,56,43), Scalar(35,77,64), mask2);
    //Mat maskRoadShadow;
    //cv::inRange(blur_imgHLS, ConvertArraytoScalar(minRoadShadowTh), ConvertArraytoScalar(maxRoadShadowTh), maskRoadShadow);
    //Mat maskRoadCombined = maskRoadNormal | maskRoadShadow;

    Mat maskLaneNormal;
    cv::inRange(imgHLS, ConvertArraytoScalar(minLaneNormalTh), ConvertArraytoScalar(maxLaneNormalTh), maskLaneNormal);
    Mat maskLaneShadow;
    cv::inRange(imgHLS, ConvertArraytoScalar(minLaneShadowTh), ConvertArraytoScalar(maxLaneShadowTh), maskLaneShadow);
    Mat maskLaneCombined = maskLaneNormal | maskLaneShadow;
    //maskRoadCombined = maskRoadCombined & CarControl::getROI();
    //cv::imshow("ThresholdRoad", maskRoadCombined);
    cv::imshow("ThresholdLane", maskLaneCombined);
    //cv::imshow("ThresholdLane", maskRoadShadow);
    //imshow("BinaryRoad", maskRoadCombined);
    //vector<vector<Point>> contours;
    //cv::findContours(maskRoadCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //cout << "test";
    //vector<Point> max_contour;
    //double max_area = 0;
    //for (const vector<Point>& contour : contours){//int i = 0; i < contours.size(); i++){
    //    //double cnt_area = cv::contourArea(contours[i]);
    //    double cnt_area = cv::contourArea(contour);
    //    if (cnt_area > max_area){
    //        max_area = cnt_area;
    //        max_contour = contour;
    //    }
    //}
    //vector<vector<Point>> max_contour_dummy;
    //max_contour_dummy.push_back(max_contour);
    //maskRoadCombined = Mat::zeros(Size(320,240), CV_8U);
    //cv::drawContours(maskRoadCombined, vector<vector<Point>>{max_contour}, -1, 255, 1);  
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    //imshow("BinaryRoad", maskRoadCombined);
    // for (int i = 0; i < points2.size(); i++)
    //imgRoad = birdViewTranform(maskRoadCombined);
    //fillLane(imgRoad);
    //imshow("Bird View Road", imgRoad);
    imgLane = birdViewTranform(maskLaneCombined);
    fillLane(imgLane);
    imshow("Bird View Lane", imgLane);
    //return dst; 
}

Mat DetectLane::laneInShadow(const Mat &imgHLS){
    Mat shadowMask, shadow, shadowHLS, laneShadow;
/*
    inRange(src, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
    Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),  
    shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);
*/
    //inRange(imgHLS, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]), 
    //    Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]), 
    //    laneShadow);

    return laneShadow;
}

void DetectLane::fillLane(Mat &src){
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ ){
        Vec4i& l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir){
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;
    //cout << rowN << endl;
    if (dir == VERTICAL){
        for (int i = 0; i <= rowN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
            //cout << i << endl;
        }
    }
    else {
        for (int i = 0; i <= colN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    //cout << res.size() << endl;
    return res;
}

vector<vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir){
    vector<std::vector<Point> > res;
    int inputN = src.size();
    //cout << inputN << endl;
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp = {};
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            res.push_back(tmp);
            continue;
        }

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 3) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                if (dir == VERTICAL) {
                    center1.y = center1.y + slideThickness*i;
                } 
                else {
                    center1.x = center1.x + slideThickness*i;
                }
                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }

    return res;
}
int countcpp = 0;
int neg_count = 0;
int neg_count_thresh = 5;
void DetectLane::detectLeftRight(const vector<vector<Point> > &points, vector<Point>& leftRaw, vector<Point>& rightRaw){
    //static vector<Point> lane1, lane2;
    vector<Point> lane1, lane2;
    //lane1.clear();
    //lane2.clear();
    
    int pointMap[points.size()][20] = {};

    int prePoint[points.size()][20] = {};
    int postPoint[points.size()][20] = {};

    int prePointX[points.size()][20] = {};
    int postPointX[points.size()][20] = {};

    int dis = 10;
    int max = 0, max2 = 0;
    Point2i posMax, posMax2;

    //memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++){
        for (int j = 0; j < points[i].size(); j++){
            prePointX[i][j] = -1;
            postPointX[i][j] = -1;

            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }
    for (int i = points.size() - 1; i >= 0; i--){
        for (int j = 0; j < points[i].size(); j++){
            int err = 320;
            for (int m = 1; m < min((int) points.size() - 1 - i, MAX_CONTINUITY_DIST + 1); m++){
                //bool check = false;
                for (int k = 0; k < points[i + m].size(); k++){
                    if (abs(points[i + m][k].x - points[i][j].x) <= dis && 
                        abs(points[i + m][k].x - points[i][j].x) < err) 
                    {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        prePointX[i][j] = i + m;
                        postPoint[i + m][k] = j;
                        postPointX[i + m][k] = i;
                        //check = true;
                    }
                }   
                //break; 
            }
            
            if (pointMap[i][j] > max){
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }
        
    if (max <= 1) return;

    for (int i = points.size() - 1; i >= 0; i--){
        for (int j = 0; j < points[i].size(); j++){
            if (pointMap[i][j] > max2 && (i > posMax.x || (i == posMax.x && j != posMax.y)) && postPoint[i][j] == -1){
                //int temp_max = pointMap[i][j];
                //while (temp_max > 1)
                max2 = pointMap[i][j];
                posMax2 = Point2i(i,j);
                /*
                int temp_max2 = pointMap[i][j];
                Point2i temp_posMax2 = Point2i(i,j);
                //lane2.clear();
                vector<Point> temp_lane2;
                while (temp_max2 > 0){
                    temp_lane2.push_back(points[temp_posMax2.x][temp_posMax2.y]);
                    //if (max2 == 1) break;                CarControl::preSteer = craftPoint(carPos, vecRightSpeed);
                    Point2i temp = temp_posMax2;
                    temp_posMax2.y = prePoint[temp.x][temp.y];
                    temp_posMax2.x = prePointX[temp.x][temp.y];        
                    temp_max2--;
                }
                //if (temp_lane2.back() != lane1.back())
                if (abs(temp_lane2.back().x - lane1.back().x) > 15 ){ 
                    max2 = temp_max2;
                    posMax2 = temp_posMax2;
                    lane2 = temp_lane2;
                }
                */
            }
        }
    }
    if (max2 <= 1) return;

    Point2i posMax_temp = posMax;
    while (max > 0){
        lane1.push_back(points[posMax_temp.x][posMax_temp.y]);
        //if (max == 1) break;
        Point2i temp = posMax_temp;
        posMax_temp.y = prePoint[temp.x][temp.y];
        posMax_temp.x = prePointX[temp.x][temp.y];        
        max--;
    }
    while (max2 > 0){
        lane2.push_back(points[posMax2.x][posMax2.y]);
        //if (max2 == 1) break;
        Point2i temp = posMax2;
        posMax2.y = prePoint[temp.x][temp.y];
        posMax2.x = prePointX[temp.x][temp.y];        
        max2--;
    }
    //if (max2 <= 1) return;

    //cout << lane1.size() << endl;
    
    
    Vec4f line1, line2;
    int lane_flag = 0;
    if (lane1.size() > 1) {
        vector<Point> subLane1(lane1.end() - min(5, lane1.size()), lane1.end());
        fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
        lane_flag++;
    }
    if (lane2.size() > 1) {
        vector<Point> subLane2(lane2.end() - min(5, lane2.size()), lane2.end());
        fitLine(subLane2, line2, 2, 0, 0.01, 0.01);
        lane_flag++;
    }
    



    //int lane_flag_left = 0, lane_flag_right = 0;
    if (lane_flag == 2){
        //lane_flag_left = 1;
        //lane_flag_right = 1;
        int lane1X = (BIRDVIEW_HEIGHT - line1[3]) * line1[0] / line1[1] + line1[2];
        int lane2X = (BIRDVIEW_HEIGHT - line2[3]) * line2[0] / line2[1] + line2[2];
        //Mat test = Mat::zeros(Size(240,320), CV_8UC3);
        //cv::circle(test, lane1[0], 1, Scalar(255,255,0), 2);
        //Point first1 = Point(0, (0-line1[2]) * line1[1] / line1[0] + line1[3]);
        //Point second1 = Point(240, (240-line1[2]) * line1[1] / line1[0] + line1[3]);
        //Point first2 = Point(0, (0-line2[2]) * line2[1] / line2[0] + line2[3]);
        //Point second2 = Point(240, (240-line2[2]) * line2[1] / line2[0] + line2[3]);
        //cv::line(test, first1, second1, Scalar(255,0,0), 2);
        //cv::line(test, first2, second2, Scalar(255,0,0), 2);
        //cout << line1 << " * " << line2 << endl;
        //cv::imshow("Test", test);
        //cout << lane1[0] << endl;
        //cout << ((line1[1] * line2[1] + line2[0] * line1[0])) << " * " << abs((line1[1] * line2[1] + line2[0] * line1[0]))  << endl;
        //cout << lane1.size() << " " << lane2.size() << endl;
        if (lane1X < lane2X){
            //if (abs(line1[0] * line2[0] + line1[1] * line2[1]) > 0.98)
            //DetectLane::vecLeft = line1;
            //DetectLane::vecRight = line2;

            //for (int i = 0; i < lane1.size(); i++)
            //{
                /*
                if (lane2[i] != DetectLane::null){
                    if (lane1[i].x > lane2[i].x) {
                        leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
                        continue;
                    }
                }
                */
            //    leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
            //}
            //for (int i = 0; i < lane2.size(); i++)
            //{
                /*
                if (lane1[i] != DetectLane::null){
                    if (lane1[i].x > lane2[i].x) {
                        rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                        continue;
                    }
                }
                */
            //    rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
            //}
            //leftLane_size = lane1.size();
            //rightLane_size = lane2.size();
            //Point rez_pnt = intersection(line1, line2);
            //if (rez_pnt == null | rez_pnt.y < maxx(lane1.back().y, lane2.back().y)){
            //    leftRaw = lane1;
            //    rightRaw = lane2;
            //} else {
        //Point rez_pnt = intersection(line1, line2);
            if (dist_point_line(lane1.back(), line2) < 5 || dist_point_line(lane2.back(), line1) < 5){
            //if (calc_X(CAR_POS_Y, line1) - CAR_POS_X > 0 && calc_X(CAR_POS_Y, line2) - CAR_POS_X < 0){
                if (angleLine(line1, line2) > 10){
                    leftRaw = lane2;
                    rightRaw = lane1;
                    //cout << "TH1<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
                }
            }
            else{
                leftRaw = lane1;
                rightRaw = lane2;
                //cout << "TH2..............................." << endl;
            }
        } else {
            if (dist_point_line(lane1.back(), line2) < 5 || dist_point_line(lane2.back(), line1) < 5){
            //if (calc_X(CAR_POS_Y, line1) - CAR_POS_X > 0 && calc_X(CAR_POS_Y, line2) - CAR_POS_X < 0){
                if (angleLine(line1, line2) > 10){
                    leftRaw = lane1;
                    rightRaw = lane2;
                    //cout << "TH3***********************" << endl;
                }
            }
            else{
                leftRaw = lane2;
                rightRaw = lane1;
                /*  
                vector<Point> subLane2(lane2.end() - min(5, lane2.size()), lane2.end());
                cout << subLane2.size() << "--" << endl;
                cout << line1 << endl;
                cout << line2 << endl;
                cout << lane1X << " " << lane2X << endl;
                cout << lane1.size() << " " << lane2.size() << endl;
                cout << angleLine(line1, line2) << endl;
                cout << dist_point_line(lane1.back(), line2) << "*" << endl;
                cout << dist_point_line(lane2.back(), line1) << "]]" << endl;
                */
            }
        }
            //DetectLane::vecLeft = line1;
            //DetectLane::vecRight = line2;
            //for (int i = 0; i < lane2.size(); i++)
            //{
                /*
                if (lane1[i] != DetectLane::null){
                    if (lane1[i].x < lane2[i].x) {
                        leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                        continue;
                    }
                }
                */
            //    leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
            //}
            //for (int i = 0; i < lane1.size(); i++)
            //{
                /*
                if (lane2[i] != DetectLane::null){
                    if (lane1[i].x < lane2[i].x) {
                        rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
                        continue;
                    }
                }
                */
             //   rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
            //}
            //Point rez_pnt = intersection(line1, line2);
            //leftRaw = lane2;
            //rightRaw = lane1;
            //leftLane_size = lane2.size();
            //rightLane_size = lane1.size();
        //}
        //cout << rightLane.size() << "*" << leftLane.size() << endl;
    }
    // Tim cach detect lane //
    /*
    else if (lane_flag == 1){
        if (lane1.size() > 1){
            if (lane1.back().x < CAR_POS_X) {
                //lane_flag_left = 1;
                leftRaw = lane1;
            }
            else if (lane1.back().x > CAR_POS_X) {
                //lane_flag_right = 1;
                rightRaw = lane1;
            }
        } 
        else{
            if (lane2.back().x < CAR_POS_X) {
                //lane_flag_left = 1;
                leftRaw = lane2;
            }
            else if (lane2.back().x > CAR_POS_X) {
                //lane_flag_right = 1;
                rightRaw = lane2;
            }
        } 
    }
    */
    //else return;
}
//int countFrame = 0;
void DetectLane::detectLane(const vector<vector<Point> > &points, vector<Point>& leftRaw, vector<Point>& rightRaw, vector<Point>& middleRaw){
    //static vector<Point> lane1, lane2;
    //cout << countFrame++ << endl;
    vector<Point> lane1, lane2, lane3;
    //lane1.clear();
    //lane2.clear();
    
    int pointMap[points.size()][20] = {};

    int prePoint[points.size()][20] = {};
    int postPoint[points.size()][20] = {};

    int prePointX[points.size()][20] = {};
    int postPointX[points.size()][20] = {};

    int dis = 10;
    int max = 0, max2 = 0, max3 = 0;
    Point2i posMax, posMax2;

    //memset(pointMap, 0, sizeof pointMap);

    for (int i = CUTOFF_HEIGHT_LANE_DETECT; i < points.size(); i++){
        for (int j = 0; j < points[i].size(); j++){
            prePointX[i][j] = -1;
            postPointX[i][j] = -1;

            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }
    for (int i = points.size() - 1; i >= CUTOFF_HEIGHT_LANE_DETECT; i--){
        for (int j = 0; j < points[i].size(); j++){
            int err = 320;
            for (int m = 1; m < min((int) points.size() - 1 - i, MAX_CONTINUITY_DIST + 1); m++){
                //bool check = false;
                for (int k = 0; k < points[i + m].size(); k++){
                    if (abs(points[i + m][k].x - points[i][j].x) <= dis && 
                        abs(points[i + m][k].x - points[i][j].x) < err) 
                    {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        prePointX[i][j] = i + m;
                        postPoint[i + m][k] = j;
                        postPointX[i + m][k] = i;
                        //check = true;
                    }
                }   
                //break; 
            }
            
            if (pointMap[i][j] > max){
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }
        
    if (max <= 1) return;

    for (int i = points.size() - 1; i >= 0; i--){
        for (int j = 0; j < points[i].size(); j++){
            if (pointMap[i][j] > max2 && (i > posMax.x || (i == posMax.x && j != posMax.y)) && postPoint[i][j] == -1){
                //int temp_max = pointMap[i][j];
                //while (temp_max > 1)
                max2 = pointMap[i][j];
                posMax2 = Point2i(i,j);
                /*
                int temp_max2 = pointMap[i][j];
                Point2i temp_posMax2 = Point2i(i,j);
                //lane2.clear();
                vector<Point> temp_lane2;
                while (temp_max2 > 0){
                    temp_lane2.push_back(points[temp_posMax2.x][temp_posMax2.y]);
                    //if (max2 == 1) break;
                    Point2i temp = temp_posMax2;
                    temp_posMax2.y = prePoint[temp.x][temp.y];
                    temp_posMax2.x = prePointX[temp.x][temp.y];        
                    temp_max2--;
                }
                //if (temp_lane2.back() != lane1.back())
                if (abs(temp_lane2.back().x - lane1.back().x) > 15 ){ 
                    max2 = temp_max2;
                    posMax2 = temp_posMax2;
                    lane2 = temp_lane2;
                }
                */
            }
        }
    }
    /*
    for (int i = points.size() - 1; i >= 0; i--){
        for (int j = 0; j < points[i].size(); j++){
            if (pointMap[i][j] > max3 && (i > posMax.x || (i == posMax.x && j != posMax.y)) && (i != posMax2.x && j != posMax2.y) && postPoint[i][j] == -1){
                //int temp_max = pointMap[i][j];
                //while (temp_max > 1)
                max3 = pointMap[i][j];
                posMax2 = Point2i(i,j);
                /*
                int temp_max2 = pointMap[i][j];
                Point2i temp_posMax2 = Point2i(i,j);
                //lane2.clear();
                vector<Point> temp_lane2;
                while (temp_max2 > 0){
                    temp_lane2.push_back(points[temp_posMax2.x][temp_posMax2.y]);
                    //if (max2 == 1) break;
                    Point2i temp = temp_posMax2;
                    temp_posMax2.y = prePoint[temp.x][temp.y];
                    temp_posMax2.x = prePointX[temp.x][temp.y];        
                    temp_max2--;
                }
                //if (temp_lane2.back() != lane1.back())
                if (abs(temp_lane2.back().x - lane1.back().x) > 15 ){ 
                    max2 = temp_max2;
                    posMax2 = temp_posMax2;
                    lane2 = temp_lane2;
                }
                *//*
            }
        }
    }
    */
    if (max2 <= 1) return;

    Point2i posMax_temp = posMax;
    vector<vector<Point>> segment_list1, segment_list2;
    vector<Point> segment_cur; //segment_pre2, segment_cur2;
    
    while (max > 0){
        lane1.push_back(points[posMax_temp.x][posMax_temp.y]);
        Point temp = posMax_temp;
        posMax_temp.y = prePoint[temp.x][temp.y];
        posMax_temp.x = prePointX[temp.x][temp.y];
        if (points[temp.x][temp.y].y >= SEGMENT_HEIGHT_LIMIT){
            segment_cur.push_back(points[temp.x][temp.y]);
            //if (max2 == 1) break;
            Point next_point;
            if (max > 1) next_point = points[posMax_temp.x][posMax_temp.y];
            if (max == 1 || dist_point_point(next_point, segment_cur.back()) >= MIN_DIST_SEGMENT) {
                segment_list1.push_back(segment_cur);
                segment_cur.clear();
                //segment_cur1.push_back(Point(posMax_temp));
                //segment_pre = segment_cur1;
            }
        }
        max--;
    }
    while (max2 > 0){
        lane2.push_back(points[posMax2.x][posMax2.y]);
        Point temp = posMax2;
        posMax2.y = prePoint[temp.x][temp.y];
        posMax2.x = prePointX[temp.x][temp.y];
        if (points[temp.x][temp.y].y >= SEGMENT_HEIGHT_LIMIT){
            segment_cur.push_back(points[temp.x][temp.y]);
            //if (max2 == 1) break;
            Point next_point;
            if (max2 > 1) next_point = points[posMax2.x][posMax2.y];
            if (max2 == 1 || dist_point_point(next_point, segment_cur.back()) >= MIN_DIST_SEGMENT) {
                segment_list2.push_back(segment_cur);
                segment_cur.clear();
                //segment_cur1.push_back(Point(posMax_temp));
                //segment_pre = segment_cur1;
            }
        }
        max2--;
    }

    Vec4f line1, line2; //dist_point_point()
    int lane_flag = 0;
    if (lane1.size() > 1) {
        vector<Point> subLane1(lane1.end() - min(5, lane1.size()), lane1.end());
        fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
        lane_flag++;
    }
    if (lane2.size() > 1) {
        vector<Point> subLane2(lane2.end() - min(5, lane2.size()), lane2.end());
        fitLine(subLane2, line2, 2, 0, 0.01, 0.01);
        lane_flag++;
    }

    bool potential_middle1, potential_middle2;
    //if(check_distance(lane1, lane2, line1, line2)) {
        potential_middle1 = check_segment_list(segment_list1);
        potential_middle2 = check_segment_list(segment_list2);
    //}
    //else {
    //    potential_middle1 = false;
    //    potential_middle2 = false;  
    //}
    //if (max2 <= 1) return;
    //bool potential_middle1 = false, potential_middle2 = false;
    //for (const vector<Point>& segment: segment_list1){
    //    if (segment.size() > LOWER_MAX_SEGMENT_SIZE) {
    //        potential_middle1 = false;
    //        break;
    //    }
    //}
    //for (const vector<Point>& segment: segment_list2){
    //    if (segment.size() > LOWER_MAX_SEGMENT_SIZE) {
    //        potential_middle2 = false;
    //        break;
    //    }
    //}
    //if (segment_list1.size() >= 4) potential_middle1 = true;
    /*
    if (segment_list1.size() > 2){
        int list_size = segment_list1.size();
        //if (list_size == 2) list_size = 3;
        int stop_index = list_size == 2 ? 0 : 1;
        if (segment_list1[0].size() >= UPPER_MAX_SEGMENT_SIZE) potential_middle1 = false;
        else{
            for (int i = list_size - 1; i >= stop_index; i--){
                if (segment_list1[i].size() <= LOWER_MAX_SEGMENT_SIZE){
                    //cout << i << " " << segment_list1[i].size() << " " << segment_list1[i + 1].size() << endl;
                    if (i > stop_index && segment_list1[i].size() > segment_list1[i - 1].size()){
                        potential_middle1 = false;
                        break;
                    }
                }
                else {
                    potential_middle1 = false;
                    break;
                }
            }
        }
        //potential_middle1 = segment_list1[list_size - 1].size() <= LOWER_MAX_SEGMENT_SIZE && segment_list1[list_size - 2].size() <= LOWER_MAX_SEGMENT_SIZE;
    }
    else potential_middle1 = false;
    //if (segment_list2.size() >= 4) potential_middle2 = true;
    if (segment_list2.size() > 2){
        int list_size = segment_list2.size();
        //if (list_size == 2) list_size = 3;
        int stop_index = list_size == 2 ? 0 : 1;
        if (segment_list2[0].size() >= UPPER_MAX_SEGMENT_SIZE) potential_middle2 = false;
        else{
            for (int i = list_size - 1; i >= stop_index; i--){
                if (segment_list2[i].size() <= LOWER_MAX_SEGMENT_SIZE){
                    //cout << i << " " << segment_list2[i].size() << " " << segment_list2[i + 1].size() << endl;
                    if (i > stop_index && segment_list2[i].size() > segment_list2[i - 1].size()){
                        potential_middle2 = false;
                        break;
                    }
                }
                else {
                    potential_middle2 = false;
                    break;
                }
            }
        }
        //potential_middle2 = segment_list2[list_size - 1].size() <= LOWER_MAX_SEGMENT_SIZE && segment_list2[list_size - 2].size() <= LOWER_MAX_SEGMENT_SIZE;
    }
    else potential_middle2 = false;
    //bool potential_middlelane_or_obstacle_or_shadow2 = segment_list2.size() > 1;
    */
    //cout << lane1.size() << endl;
    
    if (countFrame == 1027){
        cout << "HERE" << countFrame << endl;
    }
    //Vec4f line1, line2; //dist_point_point()
    //int lane_flag = 0;
    //if (lane1.size() > 1) {
    //    vector<Point> subLane1(lane1.end() - min(5, lane1.size()), lane1.end());
    //    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    //    lane_flag++;
    //}
    //if (lane2.size() > 1) {
    //    vector<Point> subLane2(lane2.end() - min(5, lane2.size()), lane2.end());
    //    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);
    //    lane_flag++;
    //}
    



    //int lane_flag_left = 0, lane_flag_right = 0;
    if (lane_flag == 2){
        //lane_flag_left = 1;
        //lane_flag_right = 1;
        int lane1X = (BIRDVIEW_HEIGHT - line1[3]) * line1[0] / line1[1] + line1[2];
        int lane2X = (BIRDVIEW_HEIGHT - line2[3]) * line2[0] / line2[1] + line2[2];
        //Mat test = Mat::zeros(Size(240,320), CV_8UC3);
        //cv::circle(test, lane1[0], 1, Scalar(255,255,0), 2);
        //Point first1 = Point(0, (0-line1[2]) * line1[1] / line1[0] + line1[3]);
        //Point second1 = Point(240, (240-line1[2]) * line1[1] / line1[0] + line1[3]);
        //Point first2 = Point(0, (0-line2[2]) * line2[1] / line2[0] + line2[3]);
        //Point second2 = Point(240, (240-line2[2]) * line2[1] / line2[0] + line2[3]);
        //cv::line(test, first1, second1, Scalar(255,0,0), 2);
        //cv::line(test, first2, second2, Scalar(255,0,0), 2);
        //cout << line1 << " * " << line2 << endl;
        //cv::imshow("Test", test);
        //cout << lane1[0] << endl;
        //cout << ((line1[1] * line2[1] + line2[0] * line1[0])) << " * " << abs((line1[1] * line2[1] + line2[0] * line1[0]))  << endl;
        //cout << lane1.size() << " " << lane2.size() << endl;
        if (lane1X < lane2X){
            //if (abs(line1[0] * line2[0] + line1[1] * line2[1]) > 0.98)
            //DetectLane::vecLeft = line1;
            //DetectLane::vecRight = line2;

            //for (int i = 0; i < lane1.size(); i++)
            //{
                /*
                if (lane2[i] != DetectLane::null){
                    if (lane1[i].x > lane2[i].x) {
                        leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
                        continue;
                    }
                }
                */
            //    leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
            //}
            //for (int i = 0; i < lane2.size(); i++)
            //{
                /*
                if (lane1[i] != DetectLane::null){
                    if (lane1[i].x > lane2[i].x) {
                        rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                        continue;
                    }
                }
                */
            //    rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
            //}
            //leftLane_size = lane1.size();
            //rightLane_size = lane2.size();
            //Point rez_pnt = intersection(line1, line2);
            //if (rez_pnt == null | rez_pnt.y < maxx(lane1.back().y, lane2.back().y)){
            //    leftRaw = lane1;
            //    rightRaw = lane2;
            //} else {
        //Point rez_pnt = intersection(line1, line2);
            if (dist_point_line(lane1.back(), line2) < 5 || dist_point_line(lane2.back(), line1) < 5){
            //if (calc_X(CAR_POS_Y, line1) - CAR_POS_X > 0 && calc_X(CAR_POS_Y, line2) - CAR_POS_X < 0){
                if (angleLine(line1, line2) > 10){
                    leftRaw = lane2;
                    rightRaw = lane1;
                    //cout << "TH1<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
                }
                is_left_left = false;
                is_right_right = false;
                //middleRaw.clear();
                //middleLaneSide = UNDEFINED;
            }
            else{
                leftRaw = lane1;
                rightRaw = lane2;
                if (potential_middle1 && !potential_middle2){
                    middleRaw = lane1;
                    middleLaneSide = LEFT;
                    is_left_left = false;
                    is_right_right = true;
                //    rightRaw = lane2;
                }
                else if (!potential_middle1 && potential_middle2){
                    middleRaw = lane2;
                    middleLaneSide = RIGHT;
                    is_left_left = true;
                    is_right_right = false;
                //    leftRaw = lane1;
                }
                //else if (potential_middle1 && potential_middle2){
                //    if (segment_list1.size() > segment_list2.size()){
                //        middleRaw = lane1;
                //    }
                //    else if (segment_list1.size() < segment_list2.size()){
                //        middleRaw = lane2;
                //    }
                //}
                else if (!potential_middle1 && !potential_middle2){
                //    leftRaw = lane1;
                //    rightRaw = lane2;
                    is_left_left = true;
                    is_right_right = true;
                    middleLaneSide = UNDEFINED;
                    middleRaw.clear();
                }
                //cout << "TH2..............................." << endl;
            }
        } else {
            if (dist_point_line(lane1.back(), line2) < 5 || dist_point_line(lane2.back(), line1) < 5){
            //if (calc_X(CAR_POS_Y, line1) - CAR_POS_X > 0 && calc_X(CAR_POS_Y, line2) - CAR_POS_X < 0){
                if (angleLine(line1, line2) > 10){
                    leftRaw = lane1;
                    rightRaw = lane2;
                    //cout << "TH3***********************" << endl;
                }
                is_left_left = false;
                is_right_right = false;
                //middleRaw.clear();
                //middleLaneSide = UNDEFINED;
            }
            else{
                leftRaw = lane2;
                rightRaw = lane1;
                if (!potential_middle1 && potential_middle2){
                    is_left_left = false;
                    is_right_right = true;
                    middleRaw = lane2;
                    middleLaneSide = LEFT;
                //    leftRaw = lane1;
                }
                else if (potential_middle1 && !potential_middle2){
                    is_left_left = true;
                    is_right_right = false;
                    middleRaw = lane1;
                    middleLaneSide = RIGHT;
                //    rightRaw = lane2;
                }
                //else if (potential_middle1 && potential_middle2){
                //    if (segment_list1.size() > segment_list2.size()){
                //        middleRaw = lane1;
                //    }
                //    else if (segment_list1.size() < segment_list2.size()){
                //        middleRaw = lane2;
                //    }
                //}
                else if (!potential_middle1 && !potential_middle2){
                //    leftRaw = lane1;
                //    rightRaw = lane2;
                    is_left_left = true;
                    is_right_right = true;
                    middleLaneSide = UNDEFINED;
                    middleRaw.clear();
                }
                /*  
                vector<Point> subLane2(lane2.end() - min(5, lane2.size()), lane2.end());
                cout << subLane2.size() << "--" << endl;
                cout << line1 << endl;
                cout << line2 << endl;
                cout << lane1X << " " << lane2X << endl;
                cout << lane1.size() << " " << lane2.size() << endl;
                cout << angleLine(line1, line2) << endl;
                cout << dist_point_line(lane1.back(), line2) << "*" << endl;
                cout << dist_point_line(lane2.back(), line1) << "]]" << endl;
                */
            }
        }
            //DetectLane::vecLeft = line1;
            //DetectLane::vecRight = line2;
            //for (int i = 0; i < lane2.size(); i++)
            //{
                /*
                if (lane1[i] != DetectLane::null){
                    if (lane1[i].x < lane2[i].x) {
                        leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                        continue;
                    }
                }
                */
            //    leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
            //}
            //for (int i = 0; i < lane1.size(); i++)
            //{
                /*
                if (lane2[i] != DetectLane::null){
                    if (lane1[i].x < lane2[i].x) {
                        rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
                        continue;
                    }
                }
                */
             //   rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
            //}
            //Point rez_pnt = intersection(line1, line2);
            //leftRaw = lane2;
            //rightRaw = lane1;
            //leftLane_size = lane2.size();
            //rightLane_size = lane1.size();
        //}
        //cout << rightLane.size() << "*" << leftLane.size() << endl;
    }
    // Tim cach detect lane //
    /*
    else if (lane_flag == 1){
        if (lane1.size() > 1){
            if (lane1.back().x < CAR_POS_X) {
                //lane_flag_left = 1;
                leftRaw = lane1;
            }
            else if (lane1.back().x > CAR_POS_X) {
                //lane_flag_right = 1;
                rightRaw = lane1;
            }
        } 
        else{
            if (lane2.back().x < CAR_POS_X) {
                //lane_flag_left = 1;
                leftRaw = lane2;
            }
            else if (lane2.back().x > CAR_POS_X) {
                //lane_flag_right = 1;
                rightRaw = lane2;
            }
        } 
    }
    */
    //else return;
}

void DetectLane::updateLeftRight(const vector<Point>& leftRaw, const vector<Point>& rightRaw, vector<Point>& left, vector<Point>& right){
    left.clear();
    right.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i++){
        left.push_back(null);
        right.push_back(null);
    }
    for (int i = 0; i < leftRaw.size(); i++){
                /*
                if (lane1[i] != DetectLane::null){
                    if (lane1[i].x < lane2[i].x) {
                        leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                        continue;
                    }
                }
                */
        left[floor(leftRaw[i].y / slideThickness)] = leftRaw[i];
    }

    for (int i = 0; i < rightRaw.size(); i++){
            /*
            if (lane1[i] != DetectLane::null){
                if (lane1[i].x < lane2[i].x) {
                    leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                    continue;
                }
            }
            */
        right[floor(rightRaw[i].y / slideThickness)] = rightRaw[i];
    }
}
void DetectLane::updateLane(const vector<Point>& leftRaw, const vector<Point>& rightRaw, const vector<Point>& middleRaw, vector<Point>& left, vector<Point>& right, vector<Point>& middle){
    left.clear();
    right.clear();
    middle.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i++){
        left.push_back(null);
        right.push_back(null);
        middle.push_back(null);
    }
    for (int i = 0; i < leftRaw.size(); i++){
                /*
                if (lane1[i] != DetectLane::null){
                    if (lane1[i].x < lane2[i].x) {
                        leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                        continue;
                    }
                }
                */
        left[floor(leftRaw[i].y / slideThickness)] = leftRaw[i];
    }

    for (int i = 0; i < rightRaw.size(); i++){
            /*
            if (lane1[i] != DetectLane::null){
                if (lane1[i].x < lane2[i].x) {
                    leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                    continue;
                }
            }
            */
        right[floor(rightRaw[i].y / slideThickness)] = rightRaw[i];
    }
    for (int i = 0; i < middleRaw.size(); i++){
            /*
            if (lane1[i] != DetectLane::null){
                if (lane1[i].x < lane2[i].x) {
                    leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                    continue;
                }
            }
            */
        middle[floor(middleRaw[i].y / slideThickness)] = middleRaw[i];
    }
}
Mat DetectLane::morphological(const Mat &img)
{
    Mat dst;

    // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
    // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );

    // blur(dst, dst, Size(3, 3));

    return dst;
}

void DetectLane::transform(Point2f* src_vertices, Point2f* dst_vertices, const Mat& src, Mat &dst){
    //Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, warpMatrix, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat DetectLane::birdViewTranform(const Mat &src)
{
    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    transform(src_vertices, dst_vertices, src, dst);
    return dst;
    //int width = src.size().width;
    //int height = src.size().height;
    /*
    Point2f src_vertices[4];
    src_vertices[0] = Point(0, SKYLINE);
    src_vertices[1] = Point(width, SKYLINE);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(105, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
    */
}
float angleVector(const Vec2f& vec1, const Vec2f& vec2){
    float vx1 = vec1[0],
          vy1 = vec1[1];
    float vx2 = vec2[0],
          vy2 = vec2[1];
    // Normalize line1 vector and line2 vector
    if (vy1 * vy2 < 0){
        vx2 = -vx2;
        vy2 = -vy2;
    }
    return acos((vx1 * vx2 + vy1 * vy2) / (sqrt(pow(vx1,2) + pow(vy1,2)) * sqrt(pow(vx2,2) + pow(vy2,2)))) / CV_PI * 180;
}
float angleLine(const Vec4f& line1, const Vec4f& line2){
    float vx1 = line1[0],
          vy1 = line1[1];
    float vx2 = line2[0],
          vy2 = line2[1];
    // Normalize line1 vector and line2 vector
    if (vy1 * vy2 < 0){
        vx2 = -vx2;
        vy2 = -vy2;
    }
    return acos((vx1 * vx2 + vy1 * vy2) / (sqrt(pow(vx1,2) + pow(vy1,2)) * sqrt(pow(vx2,2) + pow(vy2,2)))) / CV_PI * 180;
}
Vec4f createLine(const Point& pnt1, const Point& pnt2){
    float length_vec = dist_point_point(pnt1, pnt2);
    return Vec4f((pnt1.x - pnt2.x) / length_vec, (pnt1.y - pnt2.y) / length_vec, pnt1.x, pnt1.y);
}
float dist_point_line(const Point& pnt, const Vec4f& line){
    // vx, vy, x0, y0
    // (x - x0)vy - (y-y0)vx = 0
    float vx = line[0],
          vy = line[1],
          x0 = line[2],
          y0 = line[3];
    //float dist = -1;
    return abs(((pnt.x-x0) * vy - (pnt.y-y0) * vx)/sqrt(pow(vx, 2) + pow(vy, 2))); 
}
float signed_dist_point_line(const Point& pnt, const Vec4f& line){
    // vx, vy, x0, y0
    // (x - x0)vy - (y-y0)vx = 0
    float vx = line[0],
          vy = line[1],
          x0 = line[2],
          y0 = line[3];
    //float dist = -1;
    if (vy > 0) {
        vx = -vx;
        vy = -vy;
    }
    return ((pnt.x-x0) * vy - (pnt.y-y0) * vx)/sqrt(pow(vx, 2) + pow(vy, 2)); 
}
float dist_point_point(const Point& pnt1, const Point& pnt2){
    return sqrt(pow(pnt1.x-pnt2.x, 2) + pow(pnt1.y-pnt2.y, 2)); 
}
Point intersection (const Vec4f& line1, Vec4f& line2){
    float vx1 = line1[0],
          vy1 = line1[1],
          x01 = line1[2],
          y01 = line1[3];
    float vx2 = line2[0],
          vy2 = line2[1],
          x02 = line2[2],
          y02 = line2[3];
    // Normalize line1 vector and line2 vector
    if (vy1 * vy2 < 0){
        vx2 = -vx2;
        vy2 = -vy2;
    }
    if (vx1 * vx2 + vy1 * vy2 < 0.99) {
        float intersect_x = (vx2 * vy1 * x01 - vy2 * vx1 * x02 - vx1 * vx2 * (y01 - y02)) / (vx2 * vy1 - vy2 * vx1),
              intersect_y = vy1 * (intersect_x - x01) / vx1 + y01;
        return Point(intersect_x, intersect_y);
    }
    else return Point();
}
float calc_X(const int& y, const Vec4f& line){
    float vx = line[0],
          vy = line[1],
          x0 = line[2],
          y0 = line[3];
    return vx*(y-y0)/vy + x0;
}
float calc_Y(const int& x, const Vec4f& line){
    float vx = line[0],
          vy = line[1],
          x0 = line[2],
          y0 = line[3];
    return vy*(x-x0)/vx + y0;
}

Scalar ConvertArraytoScalar(int arr[3]){
    return Scalar(arr[0], arr[1], arr[2]);
}
bool check_segment_list(vector<vector<Point>>& segment_list){
    
    int list_size = segment_list.size();
    //for (int i = list_size - 1; i >= 0; i--){
    //    cout << i << " " << segment_list[i].size() << " " << segment_list[i -, 1].size() << " " << (i > 1 && ((int)segment_list[i].size() - (int) segment_list[i - 1].size() > 3)) << " - " << (int) (segment_list[i].size() - segment_list[i - 1].size()) << endl;
    //}
    //cout<<endl;
    //cout << "hehe" << endl;
    if (segment_list.size() < MIN_SEGMENT_NUMBER_REQUIREMENT || segment_list[0].size() >= UPPER_MAX_SEGMENT_SIZE) return false;
    int max_segment_size = 0;
    for (int i = list_size - 1; i >= 0; i--){
        if (segment_list[i].size() <= LOWER_MAX_SEGMENT_SIZE){
            if (segment_list[i].size() > max_segment_size) max_segment_size = segment_list[i].size();
            if (i > 1 && (int) (segment_list[i].size() - segment_list[i - 1].size()) > 3){
                //cout << "wha" << endl;
                //for (int i = list_size - 1; i >= 0; i--){
                //     cout << i << " " << segment_list[i].size() << " " << segment_list[i - 1].size() << " " << (i > 1 && ((int)segment_list[i].size() - (int) segment_list[i - 1].size() > 3)) << " - " << (int) (segment_list[i].size() - segment_list[i - 1].size()) << endl;
                //}
                return false;
            }
        }
        else return false;
    }
    //int total_pnt = 0;
    //for (int i = list_size - 1; i >= 0; i--){
    //    total_pnt += segment_list[i].size();
    //    //cout << i << " " << segment_list[i].size() << " " << segment_list[i - 1].size() << " " << (i > 1 && ((int)segment_list[i].size() - (int) segment_list[i - 1].size() > 3)) << " - " << (int) (segment_list[i].size() - segment_list[i - 1].size()) << endl;
    //}
    //cout << total_pnt << "*" << endl;

    //****Tính trung bình độ dài segment */
    //float average_pnt = total_pnt / segment_list.size();
    //if (average_pnt < 2. && average_pnt > 4) return false;

    if (max_segment_size < SEGMENT_SIZE_REQUIREMENT) return false;
    return true;
}
bool check_distance(vector<Point>& lane1, vector<Point>& lane2, Vec4f& line1, Vec4f& line2){
    if (dist_point_line(lane1.front(), line2) < DIST_SIDE_LANE_MID_LANE_UPPER_LIM && 
        dist_point_line(lane1.front(), line2) > DIST_SIDE_LANE_MID_LANE_LOWER_LIM &&
        //dist_point_line(lane1.back(), line2) < DIST_SIDE_LANE_MID_LANE_UPPER_LIM && 
        //dist_point_line(lane1.back(), line2) > DIST_SIDE_LANE_MID_LANE_LOWER_LIM &&

        dist_point_line(lane2.front(), line1) < DIST_SIDE_LANE_MID_LANE_UPPER_LIM && 
        dist_point_line(lane2.front(), line1) > DIST_SIDE_LANE_MID_LANE_LOWER_LIM &&
        dist_point_line(lane2.back(), line1) < DIST_SIDE_LANE_MID_LANE_UPPER_LIM && 
        dist_point_line(lane2.back(), line1) > DIST_SIDE_LANE_MID_LANE_LOWER_LIM
        ) return true;
    return false;
}