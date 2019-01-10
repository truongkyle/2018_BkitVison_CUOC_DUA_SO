#include "detect_obstacle.h"

#include "detect_traffic_sign.h"

cv::CascadeClassifier DetectSign::rock_detect;
Mat* DetectSign::image;
int DetectSign::init_flag = 0;
void DetectSign::store_image(Mat& image){
    DetectSign::image = &image;
}
Mat& DetectSign::get_image(){
    return *(DetectSign::image);
}
void DetectSign::init(const string& rock_dir){
    if (!rock_detect.load(rock_dir)){
        cout << "Khong load duoc file xml haar " << endl;
        init_flag = 0;
        return;
    }
    init_flag = 1;
}

Obstacle DetectSign::get_rock(bool draw){
    if (!init_flag) return ERROR;
    if (check_rock(draw)) return obstacle_coordinate;
    else return NONE;
}

bool DetectSign::check_rock(bool draw){
    vector<Rect> rock_region;
    rock_detect.detectMultiScale(DetectSign::get_image(), rock_region, 1.1, 3);
    if (!rock_region.size()) return false;
    if (draw)
        for (const Rect& region : rock_region)
            cv::rectangle(DetectSign::get_image(), region, Scalar(0,0,255), 2);
    return true;
}


