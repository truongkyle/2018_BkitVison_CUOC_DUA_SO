#include "detect_obstacle.h"

//#include "detect_traffic_sign.h"

//cv::CascadeClassifier DetectSign::rock_detect;
//Mat* DetectSign::image;
//int DetectObstacle::init_flag = 0;
void DetectObstacle::store_image(Mat& image){
    DetectObstacle::image = &image;
}
Mat& DetectObstacle::get_image(){
    return *(DetectObstacle::image);
}
void DetectObstacle::init(const string& rock_dir){
    if (!rock_detect.load(rock_dir)){
        cout << "Khong load duoc file xml haar " << endl;
        init_flag = 0;
        return;
    }
    init_flag = 1;
}

Obstacle DetectObstacle::get_rock(bool draw){
    Obstacle coordinate;
    //if (!init_flag) return ERROR;
    
    if (check_rock(coordinate, draw)) return coordinate;
}

bool DetectObstacle::check_rock(Obstacle& coordinate, bool draw){
    if (!init_flag) {
        cout << "Check obstacle haar path" << endl;
        return false;
    }
    vector<Rect> rock_region;
    rock_detect.detectMultiScale(this->get_image(), rock_region, 1.1, 3);
    if (!rock_region.size()) return false;
    if (draw)
        for (const Rect& region : rock_region)
            cv::rectangle(this->get_image(), region, Scalar(0,0,255), 2);
    coordinate.set(rock_region);
    return true;
}


