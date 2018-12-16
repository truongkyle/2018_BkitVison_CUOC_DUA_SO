#include "detect_traffic_sign.h"

cv::CascadeClassifier DetectSign::left_traffic_sign_detect;
cv::CascadeClassifier DetectSign::right_traffic_sign_detect;
Mat* DetectSign::image;
void DetectSign::store_image(Mat& image){
    DetectSign::image = &image;
}
Mat& DetectSign::get_image(){
    return *(DetectSign::image);
}
void DetectSign::init(const string& left_haar_dir, const string& right_haar_dir){
    if (!left_traffic_sign_detect.load(left_haar_dir))
        cout << "Khong load duoc file xml haar " << endl;
    if (!right_traffic_sign_detect.load(right_haar_dir))
        cout << "Khong load duoc file xml haar " << endl;
}

TRAFFIC_SIGN DetectSign::get_traffic_sign(bool draw){
    if (check_traffic_sign_left(draw)) return TURN_LEFT;
    else if (check_traffic_sign_right(draw)) return TURN_RIGHT;
    else return NONE;
}

bool DetectSign::check_traffic_sign_left(bool draw){
    vector<Rect> traffic_sign_region;
    left_traffic_sign_detect.detectMultiScale(DetectSign::get_image(), traffic_sign_region, 1.3, 1);
    if (!traffic_sign_region.size()) return false;
    if (draw)
        for (Rect region : traffic_sign_region)
            cv::rectangle(DetectSign::get_image(), region, Scalar(0,0,255), 2);
    return true;
}

bool DetectSign::check_traffic_sign_right(bool draw){
    vector<Rect> traffic_sign_region;
    //cout << cv::Size() << endl;
    right_traffic_sign_detect.detectMultiScale(DetectSign::get_image(), traffic_sign_region, 1.3, 1);
    if (!traffic_sign_region.size()) return false;
    if (draw)
        for (Rect region : traffic_sign_region)
            cv::rectangle(DetectSign::get_image(), region, Scalar(0,0,255), 2);
    return true;
}
