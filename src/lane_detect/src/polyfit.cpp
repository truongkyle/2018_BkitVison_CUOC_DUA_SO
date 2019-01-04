
/*
The Original cvPolyfit function was written By "Onkar Raut"
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Adapted By "Prasanna Kumar Routray"
*/

/***********************************************************************
Name: polyfitTest.cpp
Date: 22-05-2016
Author: Prasanna Kumar Routray
Compilation: catkin_make
cMakeLists: add_executable(polyfitTest src/polyfitTest.cpp)
target_link_libraries(polyfitTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
***********************************************************************/
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include "polyfit.h"

using namespace cv;
using namespace std;

void polyfit(const Mat &src_x, const Mat &src_y, Mat &dst, int order)
{
    CV_Assert((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1) && (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));
    Mat X;
    X = Mat::zeros(src_x.rows, order + 1, CV_32FC1);
    Mat copy;
    for (int i = 0; i <= order; i++)
    {
        copy = src_x.clone();
        pow(copy, i, copy);
        Mat M1 = X.col(i);
        copy.col(0).copyTo(M1);
    }
    Mat X_t, X_inv;
    transpose(X, X_t);
    Mat temp = X_t * X;
    Mat temp2;
    invert(temp, temp2);
    Mat temp3 = temp2 * X_t;
    Mat W = temp3 * src_y;
    //#ifdef DEBUG
        //0
        
        //cout <<"PRINTING INPUT AND OUTPUT FOR VALIDATION AGAINST MATLAB RESULTS\n";
        //cout <<"SRC_X : "<< src_x << endl;
        //cout <<"SRC_Y : "<< src_y << endl;
        //cout <<"X : "<< X << endl;
        //cout <<"X_T : "<< X_t << endl;
        //cout <<"W :"<< W << endl;
    //#endif
    W.copyTo(dst);
}
/*
void cvPolyfit(cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order)
{
    CV_FUNCNAME("cvPolyfit");
    __CV_BEGIN__;
    {
        CV_ASSERT((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1) && (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));
        Mat X;
        X = Mat::zeros(src_x.rows, order + 1, CV_32FC1);
        Mat copy;
        for (int i = 0; i <= order; i++)
        {
            copy = src_x.clone();
            pow(copy, i, copy);
            Mat M1 = X.col(i);
            copy.col(0).copyTo(M1);
        }
        Mat X_t, X_inv;
        transpose(X, X_t);
        Mat temp = X_t * X;
        Mat temp2;
        invert(temp, temp2);
        Mat temp3 = temp2 * X_t;
        Mat W = temp3 * src_y;
#ifdef DEBUG
        cout <<"PRINTING INPUT AND OUTPUT FOR VALIDATION AGAINST MATLAB RESULTS\n";
        cout <<"SRC_X : "<< src_x << endl;
        cout <<"SRC_Y : "<< src_y << endl;
        cout <<"X : "<< X << endl;
        cout <<"X_T : "<< X_t << endl;
        cout <<"W :"<< W << endl;
#endif
        dst = W.clone();
    }
}
*/
vector<float> fitPoly(const vector<Point2f> &src, int order)
{
    Mat src_x = Mat(src.size(), 1, CV_32F);
    Mat src_y = Mat(src.size(), 1, CV_32F);
    for (int i = 0; i < src.size(); i++)
    {
        src_x.at<float>(i, 0) = (float)src[i].x;
        src_y.at<float>(i, 0) = (float)src[i].y;
    }
        //cout <<"PRINTING INPUT AND OUTPUT FOR VALIDATION AGAINST MATLAB RESULTS\n";
        //cout <<"SRC_X : "<< src_x << endl;
        //cout <<"SRC_Y : "<< src_y << endl;
    Mat fit_weights(order + 1, 1, CV_32FC1);
    vector<float> fit_weight;
    polyfit(src_x, src_y, fit_weights, order);
    // Y = Cy * t^2 + By * t + Ay———–> This one is of second order (so 3 coefficients)
    for (int i = fit_weights.size().height - 1; i >= 0; i--)
    {
        float A = fit_weights.at<float>(i, 0);
        fit_weight.push_back(A);
    }
    return fit_weight;
}