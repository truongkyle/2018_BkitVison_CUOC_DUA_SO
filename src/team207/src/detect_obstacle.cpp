#include "detect_obstacle.h"
#include "detectlane.h"
#include "carcontrol.h"

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
void DetectObstacle::init(const string& rock_dir, const string& stacking_dir){
    //cout << rock_dir << stacking_dir << endl;
    if (rock_detect.load(rock_dir) && stacking_detect.load(stacking_dir)){
        init_flag = 1;
        return;
    }
    cout << "Khong load duoc file xml haar " << endl;
    init_flag = 0;
    return;
}

Obstacle DetectObstacle::get_obstacle(bool draw){
    Obstacle coordinate;
    //if (!init_flag) return ERROR;
    //bool test = check_obstacle(draw);
    if (check_obstacle(coordinate, draw)) return coordinate;
}

bool DetectObstacle::check_obstacle(Obstacle& coordinate, bool draw){
    if (!init_flag) {
        cout << "Check obstacle haar path" << endl;
        return false;
    }
    vector<Rect> rock_region;
    rock_detect.detectMultiScale(this->get_image(), rock_region, 1.1, 1);
    vector<Rect> stacking_region;
    stacking_detect.detectMultiScale(this->get_image(), stacking_region, 1.1, 1);
    if (!rock_region.size() && !stacking_region.size()) {
        //obstacle.clear();
        return false;
    }
    vector<Rect> combined;
    combined.reserve(rock_region.size() + stacking_region.size());
    combined.insert(combined.end(), rock_region.begin(), rock_region.end());
    combined.insert(combined.end(), stacking_region.begin(), stacking_region.end());
    if (draw)
        for (const Rect& region : combined)
            cv::rectangle(this->get_image(), region, Scalar(255,255,0), 2);
    coordinate.set(combined);
    //obstacle.set(combined);
    //coordinate.computeWarp();
    return true;
}

void Obstacle::calc_warp(Mat& warpMatrix){
    resultPoint.clear();
    for (const Rect& obstacle: obstacle_coordinate){
        Point bottomLeft = Point(obstacle.x, obstacle.y + obstacle.height);
        Point bottomRight = Point(obstacle.x + obstacle.width, obstacle.y + obstacle.height);
        vector<Point> transform_pnt{bottomLeft, bottomRight};
        resultPoint.push_back(unWarpPoint(transform_pnt, warpMatrix));
    }
}

Point Obstacle::offsetAngle(Point& carPos, Point &Steer, float car_width, float car_height, DetectLane& detect, bool is_steer_middle){
    if (!is_there_obstacle()) return Steer;
    if (!detect.is_left_left && !detect.is_right_right) return Steer;
    Mat warpMatrix = detect.getWarpMatrix();
    calc_warp(warpMatrix);
    int closest_obstacle_y = 0;
    int i = -1, index = -1;
    for (const vector<Point>& obstacle: resultPoint){
        i++;
        if (obstacle[0].y > closest_obstacle_y) {
            closest_obstacle_y = obstacle[0].y;
            index = i;
        }
    }
    if (index < 0) return Steer;
    int slice_num = closest_obstacle_y / slideThickness;
    if (slice_num <= BIRDVIEW_HEIGHT/slideThickness - DISTANCE_SLICE_TO_CHANGE_STEER) return Steer;
    int LEFTX, RIGHTX;
    int slice_num_left = slice_num, slice_num_right = slice_num;
    while(detect.getLeftLane()[slice_num_left] == DetectLane::null){
        slice_num_left++;
    };
    while(detect.getRightLane()[slice_num_right] == DetectLane::null){
        slice_num_right++;
    }
    if (detect.is_left_left && detect.is_right_right){
        LEFTX = detect.getLeftLane()[slice_num_left].x;
        RIGHTX = detect.getRightLane()[slice_num_right].x;
    }
    else if(detect.is_left_left && !detect.is_right_right){
        LEFTX = detect.getLeftLane()[slice_num_left].x;
        int middleX = detect.getRightLane()[slice_num_right].x;
        RIGHTX = middleX + (middleX - LEFTX);
    }
    else if((detect.is_left_left && !detect.is_right_right)){
        RIGHTX = detect.getRightLane()[slice_num_right].x;
        int middleX = detect.getLeftLane()[slice_num_left].x;
        LEFTX = middleX - (RIGHTX - middleX);
    }
    if (LEFTX >= RIGHTX) return Steer;
    int BottomLeftX = resultPoint[index][0].x,
        BottomRightX = resultPoint[index][1].x,
        BottomLeftY = resultPoint[index][0].y,
        BottomRightY = resultPoint[index][1].y;
    //if ((LEFTX - BottomLeftX) * (LEFTX - BottomRightX) >= 0 || 
    //     (BottomLeftX - RIGHTX) * (BottomRightX - RIGHTX) >= 0 ||
    //     (LEFTX - BottomLeftX) *(BottomRightX - RIGHTX) >= 0) return Steer;
    int GAP_LEFT = BottomLeftX - LEFTX,
        GAP_RIGHT = RIGHTX - BottomRightX;
    if (GAP_LEFT <= 0 && GAP_RIGHT <= 0 ||
        GAP_LEFT <= 0 && BottomRightX - LEFTX <= 0 ||
        GAP_RIGHT <= 0 && RIGHTX - BottomLeftX <= 0) return Steer;
    cout << "Dieu chinh preSteer" << endl;
    int LEFTY = detect.getLeftLane()[slice_num_left].y,
        RIGHTY = detect.getRightLane()[slice_num_right].y,
        GAP_LEFT_Y = BottomLeftY - LEFTY,
        GAP_RIGHT_Y = RIGHTY - BottomRightY;
    if (GAP_LEFT > GAP_RIGHT) {
        return Point(LEFTX + GAP_LEFT / 2, LEFTY + GAP_LEFT_Y / 2);
    } 
    else {
        return Point(RIGHTX - GAP_RIGHT / 2, RIGHTY - GAP_RIGHT_Y / 2);
    }
}

