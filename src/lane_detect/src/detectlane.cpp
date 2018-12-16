#include "carcontrol.h"
#include "detectlane.h"



int DetectLane::slideThickness = 10;
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;
Point DetectLane::null = Point();
DetectLane::DetectLane() {
    cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
}

DetectLane::~DetectLane(){}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::update(const Mat &src)
{
    //Mat edges;
    //Canny(src, edges, 100, 200);
    //cv::imshow("Canny", edges);
    Mat img = preProcess(src);
    vector<Mat> layers1 = splitLayer(img);
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
    detectLeftRight(points1);
    Mat birdView, lane;
    birdView = Mat::zeros(img.size(), CV_8UC3);
    lane = Mat::zeros(img.size(), CV_8UC3);
    for (int i = 0; i < points1.size(); i++)
     {
        for (int j = 0; j < points1[i].size(); j++)
        {
            circle(birdView, points1[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

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

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }

    imshow("Lane Detect", lane);
}

Mat DetectLane::preProcess(const Mat &src)
{
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
    cv::inRange(imgHSV, Scalar(41,5,53), Scalar(93,31,102), mask1);
    Mat mask2;
    cv::inRange(imgHSV, Scalar(20,56,43), Scalar(35,77,64), mask2);
    Mat maskShadow;
    cv::inRange(imgHSV, Scalar(70,28,12), Scalar(137,145,74), maskShadow);
    Mat maskCombined;
    cv::bitwise_or(mask1, mask2, maskCombined);
    cv::bitwise_or(maskCombined,maskShadow, maskCombined);

    vector<vector<Point>> contours;
    cv::findContours(maskCombined, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //cout << "test";
    vector<Point> max_contour;
    double max_area = 0;
    for (int i = 0; i < contours.size(); i++){
        double cnt_area = cv::contourArea(contours[i]);
        if (cnt_area > max_area){
            max_area = cnt_area;
            max_contour = contours[i];
        }
    }
    vector<vector<Point>> max_contour_dummy;
    max_contour_dummy.push_back(max_contour);
    maskCombined = Mat::zeros(Size(320,240), CV_8U);
    cv::drawContours(maskCombined, max_contour_dummy, -1, 255, 2);  
    //cv::line(maskEdge, Point(0,240), Point(0,0), 0, 4, CV::LINE_AA)
    imshow("Binary", maskCombined);
    // for (int i = 0; i < points2.size(); i++)
    dst = birdViewTranform(maskCombined);
    fillLane(dst);
    imshow("Bird View", dst);
    return dst;
}

Mat DetectLane::laneInShadow(const Mat &imgHSV)
{
    Mat shadowMask, shadow, shadowHSV, laneShadow;
/*
    inRange(src, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
    Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),  
    shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);
*/
    inRange(imgHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]), 
        Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]), 
        laneShadow);

    return laneShadow;
}

void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;
    //cout << rowN << endl;
    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
            //cout << i << endl;
        }
    }
    else 
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    //cout << res.size() << endl;
    return res;
}

vector<vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
{
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
                else
                {
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
void DetectLane::detectLeftRight(const vector<vector<Point> > &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();
    
    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }
    int pointMap[points.size()][20];

    int prePoint[points.size()][20];
    int postPoint[points.size()][20];

    int prePointX[points.size()][20];
    int postPointX[points.size()][20];

    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    //memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            prePointX[i][j] = -1;
            postPointX[i][j] = -1;

            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }
    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min((int) points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + m].size(); k++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis && 
                        abs(points[i + m][k].x - points[i][j].x) < err) 
                    {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        prePointX[i][j] = i + m;
                        postPoint[i + m][k] = j;
                        postPointX[i + m][k] = i;
                        check = true;
                    }
                }   
                break; 
            }
            
            if (pointMap[i][j] > max) 
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }
    if (max == -1) return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1) break;
        Point2i temp = posMax;
        posMax.y = prePoint[temp.x][temp.y];
        posMax.x = prePointX[temp.x][temp.y];        
        max--;
    }
    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;
        Point2i temp = posMax2;
        posMax2.y = prePoint[temp.x][temp.y];
        posMax2.x = prePointX[temp.x][temp.y];        
        max2--;
    }   
    //cout << lane1.size() << endl;
    vector<Point> subLane1(lane1.end() - 3, lane1.end());
    vector<Point> subLane2(lane2.end() - 3, lane2.end());
    Vec4f line1, line2;
    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);
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
    if (lane1X < lane2X)
    {
        DetectLane::vecLeft = line1;
        DetectLane::vecRight = line2;
        for (int i = 0; i < lane1.size(); i++)
        {
            /*
            if (lane2[i] != DetectLane::null){
                if (lane1[i].x > lane2[i].x) {
                    leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
                    continue;
                }
            }
            */
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            /*
            if (lane1[i] != DetectLane::null){
                if (lane1[i].x > lane2[i].x) {
                    rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                    continue;
                }
            }
            */
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        DetectLane::vecLeft = line1;
        DetectLane::vecRight = line2;
        for (int i = 0; i < lane2.size(); i++)
        {
            /*
            if (lane1[i] != DetectLane::null){
                if (lane1[i].x < lane2[i].x) {
                    leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
                    continue;
                }
            }
            */
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            /*
            if (lane2[i] != DetectLane::null){
                if (lane1[i].x < lane2[i].x) {
                    rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
                    continue;
                }
            }
            */
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
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

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat DetectLane::birdViewTranform(const Mat &src)
{
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
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
}

