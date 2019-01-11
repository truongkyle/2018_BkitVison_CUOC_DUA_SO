#include "detectlane.h"
#include "carcontrol.h"
#include "detect_traffic_sign.h"
#include "detect_obstacle.h"
#include "my_define.h"

#define DIST_SPEED 10
#define DIST_SPEED_B 4
#define DIST_ANGLE 2
#define DIST_ANGLE_B 1
#define LIM_DIST_SPEED 2
#define LIM_DIST_ANGLE 2
#define OFFSET_POS 8
#define TRAFFIC_SIGN_COOLDOWN 10
#define TURN_SPEED 40
//#define NORMAL_SPEED 60
#define CAR_POS_RADIUS 10
#define SLOW_SPEED 40
//#define MEDIUM_SPEED
#define FAST_SPEED 60
#define SMALL_TURN_ANGLE_UPPER_LIM 10
#define NORMAL_LANE_SIZE_LIM 15

Point craftPoint(const Point& carPos, const Vec2f& vec);
Point craftPoint(const Point& carPos, const float& angle);
Point unWarpPoint(Point& pnt, Mat& warpMatrixInv);
int cooldown = 0;
Mat* CarControl::maskROI;
Mat CarControl::maskRoiLane;
Mat CarControl::maskRoiIntersection;
Mat* CarControl::maskROI1D;
Mat CarControl::maskRoiLane1D;
Mat CarControl::maskRoiIntersection1D;
Point CarControl::preSteer;
float CarControl::preSpeed;
Vec2f CarControl::vecLeftSpeed, CarControl::vecRightSpeed, CarControl::vecLeftAngle, CarControl::vecRightAngle;
Point CarControl::LeftAboveSpeedA, CarControl::RightAboveSpeedA, CarControl::LeftBelowSpeedA, CarControl::RightBelowSpeedA;
Point CarControl::LeftAboveAngleA, CarControl::RightAboveAngleA, CarControl::LeftBelowAngleA, CarControl::RightBelowAngleA;
bool CarControl::flag_expand;
TRAFFIC_SIGN turn = NONE;
vector<Point> list_point_ROI = {Point(0,240), Point(0,150), Point(100, SKYLINE), Point(320-100, SKYLINE), Point(320,150), Point(320,240)};
vector<Point> list_point_noROI = {Point(0,240), Point(0,SKYLINE), Point(320,SKYLINE), Point(320,240)};
//vector<Point> list_point_ROI = {Point(0,150), Point(100, SKYLINE), Point(320-100, SKYLINE), Point(320,150)};
//vector<Point> list_point_noROI = {Point(0,150), Point(0,SKYLINE), Point(320,SKYLINE), Point(320,150)};
void CarControl::init(){
    maskRoiLane = Mat::zeros(Size(320,240), CV_8UC3);
    maskRoiIntersection = Mat::zeros(Size(320,240), CV_8UC3);
    maskRoiLane1D = Mat::zeros(Size(320,240), CV_8U);
    maskRoiIntersection1D = Mat::zeros(Size(320,240), CV_8U);
    cv::fillConvexPoly(maskRoiLane, list_point_ROI, Scalar(255,255,255));
    cv::fillConvexPoly(maskRoiIntersection, list_point_noROI, Scalar(255,255,255));
    cv::fillConvexPoly(maskRoiLane1D, list_point_ROI, Scalar(255,255,255));
    cv::fillConvexPoly(maskRoiIntersection1D, list_point_noROI, Scalar(255,255,255));
    //maskROI = &maskRoiIntersection;
    setROI(true);
}

CarControl::CarControl(){
    carPos.x = CAR_POS_X;
    carPos.y = CAR_POS_Y;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("team207_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("team207_speed",10);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst){
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = CV_PI;
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dy <= 0) {
        if (dx > 0) return 50;
        else if (dx < 0) return -50;
    }
    if (dx < 0) return max(-atan(-dx / dy) * 180 / pi, -50.);
    return min(atan(dx / dy) * 180 / pi, 50.);
}

float CarControl::errorAngle(const Point &pntAboveSpeed, const Point &pntBelow){
    if (pntAboveSpeed.x == pntBelow.x) return 0;
    if (pntAboveSpeed.y == pntBelow.y) return (pntAboveSpeed.x < pntBelow.x ? -90 : 90);
    double pi = CV_PI;
    double dx = pntAboveSpeed.x - pntBelow.x;
    double dy = pntBelow.y - pntAboveSpeed.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}
//static global_flag = 0;
//bool turn_flag = 0;
//static int global_cnt = 0;
//static int global_neg_cnt = 0;
//static int turn  = 0;

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity){
    /*
    int i = left.size() - 11;
    float error = preError;
    //int origin = 12;
    /*
    int cnt_null = 0;
    int flag = 1;
    for (int k = 0; k < 6; k++){
        if (left[i - k] != DetectLane::null || right[i - k] != DetectLane::null) {
            flag = 0;
            break; 
        }
    }
    //cout << "Test" << endl;
    //if (flag) {
    //    global_cnt++;
    //    global_neg_cnt = 0;
    //} else{
    //    global_cnt = 0;
    //    global_neg_cnt++;
    //}
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0) return;
    }
    Mat birdview_img = Mat::zeros(Size(240,320), CV_8UC3);
    //if (left[i])
    // Find above left
    //int i_left_above = i - 3,
    //    i_right_above = i - 3,
    //    i_left_below = i,
    //    i_right_below = i,
    //    thresh = 3;
    //while (i_left_above && thresh){
    //    if (left[i_left_above] != DetectLane::null) break;
    //    i_left_above--;
    //    thresh--;
    //}
    //thresh = 3;
    /*
    while (i_right_above){
        if (right[i_right_above] != DetectLane::null) break;
        i_right_above--;
        thresh--;
    }
    thresh = 3;
    while (i_left_below < left.size()){
        if (left[i_left_below] != DetectLane::null) break;
        i_left_below++;
        thresh--;
    }
    thresh = 3;
    while (i_right_below < right.size()){
        if (right[i_right_below] != DetectLane::null) break;
        i_right_below++;
        thresh--;
    }
    if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)   
    {
        error = errorAngle((left[i] + right[i]) / 2);
        cv::circle(birdview_img, (left[i] + right[i]) / 2, 3, Scalar(255,0,0), 3);
        cv::line(birdview_img, Point(120,300), (left[i] + right[i]) / 2, Scalar(0,255,0), 1);
    } 
    else if (left[i] != DetectLane::null)
    {
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
        cv::circle(birdview_img, left[i] + Point(laneWidth / 2, 0), 3, Scalar(255,0,0), 3);
        cv::line(birdview_img, Point(120,300), left[i] + Point(laneWidth / 2, 0), Scalar(0,255,0), 1);
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
        cv::circle(birdview_img, right[i] - Point(laneWidth / 2, 0), 3, Scalar(255,0,0), 3);
        cv::line(birdview_img, Point(120,300), right[i] - Point(laneWidth / 2, 0), Scalar(0,255,0), 1);
    }
    //cv::line(birdview_img, left[i_left_above], left[i_left_below], Scalar(255,0,0), 2);
    //cv::line(birdview_img, right[i_right_above], right[i_right_below], Scalar(0,255,0), 2);
    cv::imshow("Center road", birdview_img);
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    speed.data = 30;
    angle.data = error;
    //cout << angle.data << endl;
    //cout << "Tur" << endl;
    //cout << error << endl;
    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
    */
}

int null_count = 0;
void CarControl::driverCar(DetectLane* detect){
    vector<Point> left = detect->getLeftLane();
    vector<Point> right = detect->getRightLane();
    int i = left.size() - OFFSET_POS;
    //cout << left << " " << detect->getRightLaneSize() << endl;
    float error;// = preError;
    //int origin = 12;
    /*
    int cnt_null = 0;
    int flag = 1;
    for (int k = 0; k < 6; k++){
        if (left[i - k] != DetectLane::null || right[i - k] != DetectLane::null) {
            flag = 0;
            break; 
        }
    }*/
    //cout << "Test" << endl;
    //if (flag) {
    //    global_cnt++;
    //    global_neg_cnt = 0;
    //} else{
    //    global_cnt = 0;
    //    global_neg_cnt++;
    //}
    Mat birdview_img = Mat::zeros(Size(240,320), CV_8UC3);

    // Find above left
    //int i_left_above = i - 3,
    //    i_right_above = i - 3,
    //    i_left_below = i,
    //    i_right_below = i,
    //    thresh = 3;
    //while (i_left_above && thresh){
    //    if (left[i_left_above] != DetectLane::null) break;
    //    i_left_above--;
    //    thresh--;
    //}
    //thresh = 3;
    /*
    while (i_right_above){
        if (right[i_right_above] != DetectLane::null) break;
        i_right_above--;
        thresh--;
    }
    thresh = 3;
    while (i_left_below < left.size()){
        if (left[i_left_below] != DetectLane::null) break;
        i_left_below++;
        thresh--;
    }
    thresh = 3;
    while (i_right_below < right.size()){
        if (right[i_right_below] != DetectLane::null) break;
        i_right_below++;
        thresh--;
    }*/
    //while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
    //    i--;
    //    if (i < 0) break;
    //}
    Point LeftAboveSpeed, RightAboveSpeed, LeftBelowSpeed, RightBelowSpeed;
    Point LeftAboveAngle, RightAboveAngle, LeftBelowAngle, RightBelowAngle;
    //cout << steer << "ls" << endl
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    if (i >= 0) {
        int k = i;
        while (k < left.size()){
            if (left[k] != DetectLane::null) {
                LeftBelowSpeed = left[k];
                LeftBelowAngle = left[k];
                int temp = k;
                int found_flag1 = 0;
                int found_flag2 = 0;
                k = temp - DIST_SPEED;
                while (k >= max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    if (left[k] != DetectLane::null) {
                        LeftAboveSpeed = left[k];
                        found_flag1 = 1;
                        break;
                    }
                    k--;
                }
                k = temp - DIST_ANGLE;
                while (k >= max(temp - DIST_ANGLE - LIM_DIST_ANGLE, 0)){
                    if (left[k] != DetectLane::null) {
                        LeftAboveAngle = left[k];
                        found_flag2 = 1;
                        break;
                    }
                    k--;
                }
                k = temp + DIST_SPEED_B;
                if (!found_flag1){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    //leftBelowSpeed = left[k];
                    while (k < min(temp + DIST_SPEED_B + LIM_DIST_SPEED, (int) left.size())){
                        if (left[k] != DetectLane::null) {
                            LeftAboveSpeed = left[temp];
                            LeftBelowSpeed = left[k];
                            found_flag1 = 1;
                            break;
                        }
                        k++;
                    }
                }
                k = temp + DIST_ANGLE_B;
                if (!found_flag2){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    //LeftBelowAngle = left[k];
                    while (k < min(temp + DIST_ANGLE_B + LIM_DIST_ANGLE, (int) left.size())){
                        if (left[k] != DetectLane::null) {
                            LeftAboveAngle = left[temp];
                            LeftBelowAngle = left[k];
                            found_flag2 = 1;
                            break;
                        }
                        k++;
                    }
                }
                /*
                if (k < max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    k = temp + DIST_SPEED_BACKUP;
                    while (k < min((int) left.size(), temp + DIST_SPEED_BACKUP + LIM_DIST_SPEED_BACKUP)){
                        if (left[k] != DetectLane::null) {
                            LeftAboveSpeedSpeed = LeftBelow;
                            LeftBelow = left[k];
                            break;
                        }
                        k++;
                    }
                }
                */
                break;
            }
            k++;
        }

        k = i;
        while (k < right.size()){
            if (right[k] != DetectLane::null) {
                RightBelowSpeed = right[k];
                RightBelowAngle = right[k];
                int temp = k;
                int found_flag1 = 0;
                int found_flag2 = 0;
                k = temp - DIST_SPEED;
                while (k >= max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    if (right[k] != DetectLane::null) {
                        RightAboveSpeed = right[k];
                        found_flag1 = 1;
                        break;
                    }
                    k--;
                }
                k = temp - DIST_ANGLE;
                while (k >= max(temp - DIST_ANGLE - LIM_DIST_ANGLE, 0)){
                    if (right[k] != DetectLane::null) {
                        RightAboveAngle = right[k];
                        found_flag2 = 1;
                        break;
                    }
                    k--;
                }
                k = temp + DIST_SPEED_B;
                if (!found_flag1){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    //leftBelowSpeed = left[k];
                    while (k < min(temp + DIST_SPEED_B + LIM_DIST_SPEED, (int) right.size())){
                        if (right[k] != DetectLane::null) {
                            RightAboveSpeed = right[temp];
                            RightBelowSpeed = right[k];
                            found_flag1 = 1;
                            break;
                        }
                        k++;
                    }
                }
                k = temp + DIST_ANGLE_B;
                if (!found_flag2){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    RightBelowAngle = left[k];
                    while (k < min(temp + DIST_ANGLE_B + LIM_DIST_ANGLE, (int) right.size())){
                        if (right[k] != DetectLane::null) {
                            RightAboveAngle = right[temp];
                            RightBelowAngle = right[k];
                            found_flag2 = 1;
                            break;
                        }
                        k++;
                    }
                }
                /*
                if (k < max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    k = temp + DIST_SPEED_BACKUP;
                    while (k < min((int) right.size(), temp + DIST_SPEED_BACKUP + LIM_DIST_SPEED_BACKUP)){
                        if (right[k] != DetectLane::null) {
                            RightAboveSpeed = RightBelow;
                            RightBelow = right[k];
                            break;
                        }
                        k++;
                    }
                }
                */
                break;
            }
            k++;
        }
        if (LeftAboveSpeed != DetectLane::null && LeftBelowSpeed != DetectLane::null){
            LeftBelowSpeedA = LeftBelowSpeed;
            LeftAboveSpeedA = LeftAboveSpeed;
        }
        if (LeftAboveAngle != DetectLane::null && LeftBelowAngle != DetectLane::null){
            LeftAboveAngleA = LeftAboveAngle;
            LeftBelowAngleA = LeftBelowAngle;
        }
        if (RightAboveSpeed != DetectLane::null && RightBelowSpeed != DetectLane::null){
            RightAboveSpeedA = RightAboveSpeed;
            RightBelowSpeedA = RightBelowSpeed;
        }
        if (RightAboveAngle != DetectLane::null && RightBelowAngle != DetectLane::null){
            RightAboveAngleA = RightAboveAngle;
            RightBelowAngleA = RightBelowAngle;
        }
            /*
            else{
                speed.data = 60;
                if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
                    steer = (left[i] + right[i]) / 2;
                else if (left[i] != DetectLane::null)
                    steer = left[i] + Point(laneWidth / 2, 0);
                else
                    steer = right[i] - Point(laneWidth / 2, 0);
                maskROI = &maskRoiLane;
            }
            */
        //preSpeed = speed.data;
        //preSteer = steer;
    }
    //else {
    //    steer = preSteer;
    //    speed.data = preSpeed;
    //}
        /*
        if(flag){
            flag = 0;
            for(int k = 15; k < left.size(); k++){
                if (left[k] != DetectLane::null){
                    error = errorAngle(left[i] + Point(laneWidth / 2, 0));
                    CarControl::preError = error;
                    flag = 1;
                    break;
                }
            }
        }
        */
        
        //if (!flag) angle.data = CarControl::preError + 10;

    //cv::line(birdview_img, left[i_left_above], left[i_left_below], Scalar(255,0,0), 2);
    //cv::line(birdview_img, right[i_right_above], right[i_right_below], Scalar(0,255,0), 2);*/

    //cout << vecLeft[0] * vecRight[0] + vecLeft[1] * vecRight[1] << endl;
    //if (left[i] == DetectLane::null && right[i] == DetectLane::null) null_count++;
    //else null_count = 0;
    //if (null_count > 10) cout << "sap toi nga re" << endl;

    //cout << steer << " * " << left[i] << " * " << right[i] << endl;
    //if (preSteer == DetectLane::null){
    //    cout << "dummy" << endl;
    //}
    //cout << flag_expand << get_list_point_ROI() << endl;
    vecLeftSpeed = Vec2f(LeftAboveSpeedA.x - LeftBelowSpeedA.x, LeftAboveSpeedA.y - LeftBelowSpeedA.y);
    vecLeftSpeed = vecLeftSpeed / cv::norm(vecLeftSpeed);
    vecRightSpeed = Vec2f(RightAboveSpeedA.x - RightBelowSpeedA.x, RightAboveSpeedA.y - RightBelowSpeedA.y);
    vecRightSpeed = vecRightSpeed / cv::norm(vecRightSpeed);
    vecLeftAngle = Vec2f(LeftAboveAngleA.x - LeftBelowAngleA.x, LeftAboveAngleA.y - LeftBelowAngleA.y);
    vecLeftAngle = vecLeftAngle / cv::norm(vecLeftAngle);   
    vecRightAngle = Vec2f(RightAboveAngleA.x - RightBelowAngleA.x, RightAboveAngleA.y - RightBelowAngleA.y);
    vecRightAngle = vecRightAngle / cv::norm(vecRightAngle);
    
    #if TURN_METHOD == VECTOR_TURN
        if (vecLeftSpeed.dot(vecRightSpeed) < 0.995 ||
            vecLeftAngle.dot(vecRightAngle) < 0.995 ||
            vecLeftAngle.dot(vecLeftSpeed) < 0.995 ||
            vecRightAngle.dot(vecRightSpeed) < 0.995 ||
            (detect->getLeftLaneRaw().size() <= NORMAL_LANE_SIZE_LIM && detect->getRightLaneRaw().size() <= NORMAL_LANE_SIZE_LIM)){
            cout << vecLeftSpeed.dot(vecRightSpeed) << " " << vecLeftAngle.dot(vecRightAngle) << endl;
            cout << "Sap toi nga re" << null_count++ << endl;
            
            CarControl::preSpeed = TURN_SPEED;
            TRAFFIC_SIGN temp;
            temp = DetectSign::get_traffic_sign(true);
            //temp = TURN_RIGHT;
            //temp = TURN_LEFT;
            //temp = TURN_RIGHT;
            if (temp == ERROR) {
                cout << "Check haar path" << endl;
                return;
            }
            else if (temp != NONE) {
                turn = temp;
                cooldown = TRAFFIC_SIGN_COOLDOWN;
            }
            if (cooldown > 0 && turn != NONE) cooldown = TRAFFIC_SIGN_COOLDOWN;
            //else if (!cooldown) turn = NONE;
            if (turn == NONE) {
                cout << "Di thang" << endl;
                
                if (detect->there_is_middle_lane()) {
                    vector<Point> middle = detect->getMiddleLane();
                    //    //Point middleLanePnt;
                    if (i >= 0) {
                        int k = i;        
                        while (k < middle.size()){
                            if (middle[k] != DetectLane::null) {
                                //cout << "hi*********************************************************" << endl;
                                CarControl::preSteer = middle[k] ;
                                break;
                            }
                            k++;
                        }
                //        //= middleLanePnt;
                    }
                //    //else CarControl::preSteer = LeftBelowAngleA;
                }
                //else if (detect->getMiddleLaneSide() == RIGHT) CarControl::preSteer = RightBelowAngleA;
                else CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            }
            else if(turn == TURN_LEFT)  {
                cout << "Re trai" << endl;
                float min_vec = min(min(vecRightAngle[0], vecLeftAngle[0]), min(vecRightSpeed[0], vecLeftSpeed[0]));
                if (min_vec > 0){
                    CarControl::preSteer = craftPoint(carPos, -50.);
                }
                else{
                if (min_vec == vecLeftAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecLeftAngle);
                else if (min_vec == vecLeftSpeed[0]) 
                    CarControl::preSteer = craftPoint(carPos, vecLeftSpeed);
                else if (min_vec == vecRightAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                else 
                    CarControl::preSteer = craftPoint(carPos, vecRightSpeed);
                }
                //if (vecLeftAngle[0] < vecRightAngle[0]) ;
                //else CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                //if (dist_point_point(CarControl::preSteer, carPos) < CAR_POS_RADIUS) 
                //LeftBelowAngleA + Point(laneWidth / 2, 0);
                //if (dist_point_point(CarControl::preSteer, carPos) < CAR_POS_RADIUS) CarControl::preSteer = Point(120 - 20, 300 - 10);
                //cout << preSteer << endl;
                //cout << min(errorAngle(CarControl::preSteer), (float) 50 *(errorAngle(CarControl::preSteer) > 0)) << endl;
            }
            else  {
                cout << "Re phai" << endl;
                float max_vec = max(max(vecRightAngle[0], vecLeftAngle[0]), max(vecRightSpeed[0], vecLeftSpeed[0]));
                if (max_vec < 0){
                    CarControl::preSteer = craftPoint(carPos, 50.);
                }
                else{
                if (max_vec == vecRightAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                else if (max_vec == vecRightSpeed[0]) 
                    CarControl::preSteer = craftPoint(carPos, vecRightSpeed);
                else if (max_vec == vecLeftAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecLeftAngle);
                else 
                    CarControl::preSteer = craftPoint(carPos, vecLeftSpeed);
                }
                // Post 50 when center angle too clese
                //cout << preSteer << endl;
                //cout << min(errorAngle(CarControl::preSteer), (float) 50 *(errorAngle(CarControl::preSteer) > 0)) << endl;
                //if (vecLeftAngle[0] < vecRightAngle[0]) CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                //else CarControl::preSteer = craftPoint(carPos, vecLeftAngle);
                //cout << vecLeftAngle << vecRightAngle << errorAngle(preSteer) << endl;
                //CarControl::preSteer = RightBelowAngleA - Point(laneWidth / 2, 0);
                //if (dist_point_point(CarControl::preSteer, carPos) < CAR_POS_RADIUS) CarControl::preSteer = Point(120 + 20, 300 - 10);
            }
            //CarControl::maskROI = &maskRoiIntersection;
            setROI(true);
        }
        else {
            if (detect->there_is_middle_lane()) {
                vector<Point> middle = detect->getMiddleLane();
                //    //Point middleLanePnt;
                if (i >= 0) {
                    int k = i;        
                    while (k < middle.size()){
                        if (middle[k] != DetectLane::null) {
                            CarControl::preSteer = middle[k];
                            //cout << "hi*********************************************************" << endl;
                            break;
                        }
                        k++;
                    }
            //        //= middleLanePnt;
                }
            //    //else CarControl::preSteer = LeftBelowAngleA;
            }
            //else if (detect->getMiddleLaneSide() == RIGHT) CarControl::preSteer = RightBelowAngleA;
            else CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            
            if (abs(errorAngle(CarControl::preSteer)) < SMALL_TURN_ANGLE_UPPER_LIM) CarControl::preSpeed = FAST_SPEED;
            else CarControl::preSpeed = SLOW_SPEED;
            //cout << abs(error) << endl;
            //CarControl::preSpeed = NORMAL_SPEED;
            //CarControl::maskROI = &maskRoiLane;
            setROI(false);
            //turn = NONE;
            if (cooldown > 0) cooldown--;
            else if (!cooldown) turn = NONE;
            /*
            if (RightBelow != DetectLane::null && LeftBelow != DetectLane::null)
                CarControl::preSteer = (RightBelow + LeftBelow) / 2;
            else if (RightBelow != DetectLane::null)
                CarControl::preSteer = RightBelow - Point(laneWidth / 2, 0);
            else if (LeftBelow != DetectLane::null)
                CarControl::preSteer = LeftBelow + Point(laneWidth / 2, 0);
            */
        }
    #elif TURN_METHOD == STICK_LANE
        if (vecLeftSpeed.dot(vecRightSpeed) < 0.99 ||
            vecLeftAngle.dot(vecRightAngle) < 0.99 ||
            vecLeftAngle.dot(vecLeftSpeed) < 0.99 ||
            vecRightAngle.dot(vecRightSpeed) < 0.99 ||
            (detect->getLeftLaneRaw().size() <= NORMAL_LANE_SIZE_LIM && detect->getRightLaneRaw().size() <= NORMAL_LANE_SIZE_LIM)){
            cout << vecLeftSpeed.dot(vecRightSpeed) << " " << vecLeftAngle.dot(vecRightAngle) << endl;
            cout << "Sap toi nga re" << null_count++ << endl;
            CarControl::preSpeed = TURN_SPEED;
            TRAFFIC_SIGN temp;
            temp = DetectSign::get_traffic_sign(true);
            //temp = TURN_RIGHT;
            //temp = TURN_LEFT;
            //temp = TURN_RIGHT;
            if (temp == ERROR) {
                cout << "Check haar path" << endl;
                return;
            }
            else if (temp != NONE) {
                turn = temp;
                cooldown = TRAFFIC_SIGN_COOLDOWN;
            }
            if (cooldown > 0 && turn != NONE) cooldown = TRAFFIC_SIGN_COOLDOWN;
            //else if (!cooldown) turn = NONE;
            if (turn == NONE) {
                cout << "Di thang" << endl;
                CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            }
            else if(turn == TURN_LEFT)  {
                cout << "Re trai" << endl;
                CarControl::preSteer = LeftBelowAngleA + Point(laneWidth / 2, 0);
            }
            else  {
                cout << "Re phai" << endl;
                CarControl::preSteer = RightBelowAngleA - Point(laneWidth / 2, 0);
            }
            //CarControl::maskROI = &maskRoiIntersection;
            setROI(true);
        }
        else {
            CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            if (abs(errorAngle(CarControl::preSteer)) < SMALL_TURN_ANGLE_UPPER_LIM) CarControl::preSpeed = FAST_SPEED;
            else CarControl::preSpeed = SLOW_SPEED;
            setROI(false);
            //turn = NONE;
            if (cooldown > 0) cooldown--;
            else if (!cooldown) turn = NONE;
        }
    #endif
    //cout << turn << endl;
    //cout << CarControl::preSteer << endl;
    Mat test = Mat::zeros(Size(240,320), CV_8UC3);
    cv::line(test, RightAboveSpeedA, RightBelowSpeedA, Scalar(255,0,0), 2);
    cv::line(test, LeftAboveSpeedA, LeftBelowSpeedA, Scalar(0,255,0), 2);
    cv::line(test, RightAboveAngleA, RightBelowAngleA, Scalar(255,255,0), 2);
    cv::line(test, LeftAboveAngleA, LeftBelowAngleA, Scalar(0,255,255), 2);
    cv::imshow("Test", test);
    
    error = errorAngle(CarControl::preSteer);
    //cout << "sadad" << error << endl;
    angle.data = error;
    speed.data = CarControl::preSpeed;
    //cout << angle.data << endl;
    //cout << "Tur" << endl;
    //cout << error << endl;
    cv::circle(birdview_img, CarControl::preSteer, 3, Scalar(255,0,0), 3);
    cv::line(birdview_img, Point(120,300), CarControl::preSteer, Scalar(0,255,0), 1);
    cv::imshow("Center road", birdview_img);
    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
    return;
}

void CarControl::driverCar(DetectLane* detect, DetectObstacle* obstacle){
    vector<Point> left = detect->getLeftLane();
    vector<Point> right = detect->getRightLane();
    int i = left.size() - OFFSET_POS;
    //cout << left << " " << detect->getRightLaneSize() << endl;
    float error;// = preError;
    //int origin = 12;
    /*
    int cnt_null = 0;
    int flag = 1;
    for (int k = 0; k < 6; k++){
        if (left[i - k] != DetectLane::null || right[i - k] != DetectLane::null) {
            flag = 0;
            break; 
        }
    }*/
    //cout << "Test" << endl;
    //if (flag) {
    //    global_cnt++;
    //    global_neg_cnt = 0;
    //} else{
    //    global_cnt = 0;
    //    global_neg_cnt++;
    //}
    Mat birdview_img = Mat::zeros(Size(240,320), CV_8UC3);

    // Find above left
    //int i_left_above = i - 3,
    //    i_right_above = i - 3,
    //    i_left_below = i,
    //    i_right_below = i,
    //    thresh = 3;
    //while (i_left_above && thresh){
    //    if (left[i_left_above] != DetectLane::null) break;
    //    i_left_above--;
    //    thresh--;
    //}
    //thresh = 3;
    /*
    while (i_right_above){
        if (right[i_right_above] != DetectLane::null) break;
        i_right_above--;
        thresh--;
    }
    thresh = 3;
    while (i_left_below < left.size()){
        if (left[i_left_below] != DetectLane::null) break;
        i_left_below++;
        thresh--;
    }
    thresh = 3;
    while (i_right_below < right.size()){
        if (right[i_right_below] != DetectLane::null) break;
        i_right_below++;
        thresh--;
    }*/
    //while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
    //    i--;
    //    if (i < 0) break;
    //}
    Point LeftAboveSpeed, RightAboveSpeed, LeftBelowSpeed, RightBelowSpeed;
    Point LeftAboveAngle, RightAboveAngle, LeftBelowAngle, RightBelowAngle;
    //cout << steer << "ls" << endl
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    if (i >= 0) {
        int k = i;
        while (k < left.size()){
            if (left[k] != DetectLane::null) {
                LeftBelowSpeed = left[k];
                LeftBelowAngle = left[k];
                int temp = k;
                int found_flag1 = 0;
                int found_flag2 = 0;
                k = temp - DIST_SPEED;
                while (k >= max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    if (left[k] != DetectLane::null) {
                        LeftAboveSpeed = left[k];
                        found_flag1 = 1;
                        break;
                    }
                    k--;
                }
                k = temp - DIST_ANGLE;
                while (k >= max(temp - DIST_ANGLE - LIM_DIST_ANGLE, 0)){
                    if (left[k] != DetectLane::null) {
                        LeftAboveAngle = left[k];
                        found_flag2 = 1;
                        break;
                    }
                    k--;
                }
                k = temp + DIST_SPEED_B;
                if (!found_flag1){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    //leftBelowSpeed = left[k];
                    while (k < min(temp + DIST_SPEED_B + LIM_DIST_SPEED, (int) left.size())){
                        if (left[k] != DetectLane::null) {
                            LeftAboveSpeed = left[temp];
                            LeftBelowSpeed = left[k];
                            found_flag1 = 1;
                            break;
                        }
                        k++;
                    }
                }
                k = temp + DIST_ANGLE_B;
                if (!found_flag2){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    //LeftBelowAngle = left[k];
                    while (k < min(temp + DIST_ANGLE_B + LIM_DIST_ANGLE, (int) left.size())){
                        if (left[k] != DetectLane::null) {
                            LeftAboveAngle = left[temp];
                            LeftBelowAngle = left[k];
                            found_flag2 = 1;
                            break;
                        }
                        k++;
                    }
                }
                /*
                if (k < max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    k = temp + DIST_SPEED_BACKUP;
                    while (k < min((int) left.size(), temp + DIST_SPEED_BACKUP + LIM_DIST_SPEED_BACKUP)){
                        if (left[k] != DetectLane::null) {
                            LeftAboveSpeedSpeed = LeftBelow;
                            LeftBelow = left[k];
                            break;
                        }
                        k++;
                    }
                }
                */
                break;
            }
            k++;
        }

        k = i;
        while (k < right.size()){
            if (right[k] != DetectLane::null) {
                RightBelowSpeed = right[k];
                RightBelowAngle = right[k];
                int temp = k;
                int found_flag1 = 0;
                int found_flag2 = 0;
                k = temp - DIST_SPEED;
                while (k >= max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    if (right[k] != DetectLane::null) {
                        RightAboveSpeed = right[k];
                        found_flag1 = 1;
                        break;
                    }
                    k--;
                }
                k = temp - DIST_ANGLE;
                while (k >= max(temp - DIST_ANGLE - LIM_DIST_ANGLE, 0)){
                    if (right[k] != DetectLane::null) {
                        RightAboveAngle = right[k];
                        found_flag2 = 1;
                        break;
                    }
                    k--;
                }
                k = temp + DIST_SPEED_B;
                if (!found_flag1){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    //leftBelowSpeed = left[k];
                    while (k < min(temp + DIST_SPEED_B + LIM_DIST_SPEED, (int) right.size())){
                        if (right[k] != DetectLane::null) {
                            RightAboveSpeed = right[temp];
                            RightBelowSpeed = right[k];
                            found_flag1 = 1;
                            break;
                        }
                        k++;
                    }
                }
                k = temp + DIST_ANGLE_B;
                if (!found_flag2){
                    //found_flag1 = 0;
                    //leftAboveSpeed = left[temp];
                    RightBelowAngle = left[k];
                    while (k < min(temp + DIST_ANGLE_B + LIM_DIST_ANGLE, (int) right.size())){
                        if (right[k] != DetectLane::null) {
                            RightAboveAngle = right[temp];
                            RightBelowAngle = right[k];
                            found_flag2 = 1;
                            break;
                        }
                        k++;
                    }
                }
                /*
                if (k < max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    k = temp + DIST_SPEED_BACKUP;
                    while (k < min((int) right.size(), temp + DIST_SPEED_BACKUP + LIM_DIST_SPEED_BACKUP)){
                        if (right[k] != DetectLane::null) {
                            RightAboveSpeed = RightBelow;
                            RightBelow = right[k];
                            break;
                        }
                        k++;
                    }
                }
                */
                break;
            }
            k++;
        }
        if (LeftAboveSpeed != DetectLane::null && LeftBelowSpeed != DetectLane::null){
            LeftBelowSpeedA = LeftBelowSpeed;
            LeftAboveSpeedA = LeftAboveSpeed;
        }
        if (LeftAboveAngle != DetectLane::null && LeftBelowAngle != DetectLane::null){
            LeftAboveAngleA = LeftAboveAngle;
            LeftBelowAngleA = LeftBelowAngle;
        }
        if (RightAboveSpeed != DetectLane::null && RightBelowSpeed != DetectLane::null){
            RightAboveSpeedA = RightAboveSpeed;
            RightBelowSpeedA = RightBelowSpeed;
        }
        if (RightAboveAngle != DetectLane::null && RightBelowAngle != DetectLane::null){
            RightAboveAngleA = RightAboveAngle;
            RightBelowAngleA = RightBelowAngle;
        }
            /*
            else{
                speed.data = 60;
                if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
                    steer = (left[i] + right[i]) / 2;
                else if (left[i] != DetectLane::null)
                    steer = left[i] + Point(laneWidth / 2, 0);
                else
                    steer = right[i] - Point(laneWidth / 2, 0);
                maskROI = &maskRoiLane;
            }
            */
        //preSpeed = speed.data;
        //preSteer = steer;
    }
    //else {
    //    steer = preSteer;
    //    speed.data = preSpeed;
    //}
        /*
        if(flag){
            flag = 0;
            for(int k = 15; k < left.size(); k++){
                if (left[k] != DetectLane::null){
                    error = errorAngle(left[i] + Point(laneWidth / 2, 0));
                    CarControl::preError = error;
                    flag = 1;
                    break;
                }
            }
        }
        */
        
        //if (!flag) angle.data = CarControl::preError + 10;

    //cv::line(birdview_img, left[i_left_above], left[i_left_below], Scalar(255,0,0), 2);
    //cv::line(birdview_img, right[i_right_above], right[i_right_below], Scalar(0,255,0), 2);*/

    //cout << vecLeft[0] * vecRight[0] + vecLeft[1] * vecRight[1] << endl;
    //if (left[i] == DetectLane::null && right[i] == DetectLane::null) null_count++;
    //else null_count = 0;
    //if (null_count > 10) cout << "sap toi nga re" << endl;

    //cout << steer << " * " << left[i] << " * " << right[i] << endl;
    //if (preSteer == DetectLane::null){
    //    cout << "dummy" << endl;
    //}
    //cout << flag_expand << get_list_point_ROI() << endl;
    vecLeftSpeed = Vec2f(LeftAboveSpeedA.x - LeftBelowSpeedA.x, LeftAboveSpeedA.y - LeftBelowSpeedA.y);
    vecLeftSpeed = vecLeftSpeed / cv::norm(vecLeftSpeed);
    vecRightSpeed = Vec2f(RightAboveSpeedA.x - RightBelowSpeedA.x, RightAboveSpeedA.y - RightBelowSpeedA.y);
    vecRightSpeed = vecRightSpeed / cv::norm(vecRightSpeed);
    vecLeftAngle = Vec2f(LeftAboveAngleA.x - LeftBelowAngleA.x, LeftAboveAngleA.y - LeftBelowAngleA.y);
    vecLeftAngle = vecLeftAngle / cv::norm(vecLeftAngle);   
    vecRightAngle = Vec2f(RightAboveAngleA.x - RightBelowAngleA.x, RightAboveAngleA.y - RightBelowAngleA.y);
    vecRightAngle = vecRightAngle / cv::norm(vecRightAngle);
    
    //bool is_steer_middle = false;
    #if TURN_METHOD == VECTOR_TURN
        if (vecLeftSpeed.dot(vecRightSpeed) < 0.995 ||
            vecLeftAngle.dot(vecRightAngle) < 0.995 ||
            vecLeftAngle.dot(vecLeftSpeed) < 0.995 ||
            vecRightAngle.dot(vecRightSpeed) < 0.995 ||
            (detect->getLeftLaneRaw().size() <= NORMAL_LANE_SIZE_LIM && detect->getRightLaneRaw().size() <= NORMAL_LANE_SIZE_LIM)){
            cout << vecLeftSpeed.dot(vecRightSpeed) << " " << vecLeftAngle.dot(vecRightAngle) << endl;
            cout << "Sap toi nga re" << null_count++ << endl;
            
            CarControl::preSpeed = TURN_SPEED;
            TRAFFIC_SIGN temp;
            temp = DetectSign::get_traffic_sign(true);
            //temp = TURN_RIGHT;
            //temp = TURN_LEFT;
            //temp = TURN_RIGHT;
            if (temp == ERROR) {
                cout << "Check haar path" << endl;
                return;
            }
            else if (temp != NONE) {
                turn = temp;
                cooldown = TRAFFIC_SIGN_COOLDOWN;
            }
            if (cooldown > 0 && turn != NONE) cooldown = TRAFFIC_SIGN_COOLDOWN;
            //else if (!cooldown) turn = NONE;
            if (turn == NONE) {
                cout << "Di thang" << endl;
                
                if (detect->there_is_middle_lane()) {
                    vector<Point> middle = detect->getMiddleLane();
                    //    //Point middleLanePnt;
                    if (i >= 0) {
                        int k = i;        
                        while (k < middle.size()){
                            if (middle[k] != DetectLane::null) {
                                //cout << "hi*********************************************************" << endl;
                                is_steer_middle = true;
                                CarControl::preSteer = middle[k] ;
                                break;
                            }
                            k++;
                        }
                //        //= middleLanePnt;
                    }
                //    //else CarControl::preSteer = LeftBelowAngleA;
                }
                //else if (detect->getMiddleLaneSide() == RIGHT) CarControl::preSteer = RightBelowAngleA;
                else {
                    is_steer_middle = false;
                    CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
                }
            }
            else if(turn == TURN_LEFT)  {
                cout << "Re trai" << endl;
                is_steer_middle = false;
                float min_vec = min(min(vecRightAngle[0], vecLeftAngle[0]), min(vecRightSpeed[0], vecLeftSpeed[0]));
                if (min_vec > 0){
                    CarControl::preSteer = craftPoint(carPos, -50.);
                }
                else{
                if (min_vec == vecLeftAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecLeftAngle);
                else if (min_vec == vecLeftSpeed[0]) 
                    CarControl::preSteer = craftPoint(carPos, vecLeftSpeed);
                else if (min_vec == vecRightAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                else 
                    CarControl::preSteer = craftPoint(carPos, vecRightSpeed);
                }
                //if (vecLeftAngle[0] < vecRightAngle[0]) ;
                //else CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                //if (dist_point_point(CarControl::preSteer, carPos) < CAR_POS_RADIUS) 
                //LeftBelowAngleA + Point(laneWidth / 2, 0);
                //if (dist_point_point(CarControl::preSteer, carPos) < CAR_POS_RADIUS) CarControl::preSteer = Point(120 - 20, 300 - 10);
                //cout << preSteer << endl;
                //cout << min(errorAngle(CarControl::preSteer), (float) 50 *(errorAngle(CarControl::preSteer) > 0)) << endl;
            }
            else  {
                cout << "Re phai" << endl;
                is_steer_middle = false;
                float max_vec = max(max(vecRightAngle[0], vecLeftAngle[0]), max(vecRightSpeed[0], vecLeftSpeed[0]));
                if (max_vec < 0){
                    CarControl::preSteer = craftPoint(carPos, 50.);
                }
                else{
                if (max_vec == vecRightAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                else if (max_vec == vecRightSpeed[0]) 
                    CarControl::preSteer = craftPoint(carPos, vecRightSpeed);
                else if (max_vec == vecLeftAngle[0])
                    CarControl::preSteer = craftPoint(carPos, vecLeftAngle);
                else 
                    CarControl::preSteer = craftPoint(carPos, vecLeftSpeed);
                }
                // Post 50 when center angle too clese
                //cout << preSteer << endl;
                //cout << min(errorAngle(CarControl::preSteer), (float) 50 *(errorAngle(CarControl::preSteer) > 0)) << endl;
                //if (vecLeftAngle[0] < vecRightAngle[0]) CarControl::preSteer = craftPoint(carPos, vecRightAngle);
                //else CarControl::preSteer = craftPoint(carPos, vecLeftAngle);
                //cout << vecLeftAngle << vecRightAngle << errorAngle(preSteer) << endl;
                //CarControl::preSteer = RightBelowAngleA - Point(laneWidth / 2, 0);
                //if (dist_point_point(CarControl::preSteer, carPos) < CAR_POS_RADIUS) CarControl::preSteer = Point(120 + 20, 300 - 10);
            }
            //CarControl::maskROI = &maskRoiIntersection;
            setROI(true);
        }
        else {
            if (detect->there_is_middle_lane()) {
                vector<Point> middle = detect->getMiddleLane();
                //    //Point middleLanePnt;
                if (i >= 0) {
                    int k = i;        
                    while (k < middle.size()){
                        if (middle[k] != DetectLane::null) {
                            is_steer_middle = true;
                            CarControl::preSteer = middle[k];
                            //cout << "hi*********************************************************" << endl;
                            break;
                        }
                        k++;
                    }
            //        //= middleLanePnt;
                }
            //    //else CarControl::preSteer = LeftBelowAngleA;
            }
            //else if (detect->getMiddleLaneSide() == RIGHT) CarControl::preSteer = RightBelowAngleA;
            else {
                is_steer_middle = false;
                CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            }
            
            if (abs(errorAngle(CarControl::preSteer)) < SMALL_TURN_ANGLE_UPPER_LIM) CarControl::preSpeed = FAST_SPEED;
            else CarControl::preSpeed = SLOW_SPEED;
            //cout << abs(error) << endl;
            //CarControl::preSpeed = NORMAL_SPEED;
            //CarControl::maskROI = &maskRoiLane;
            setROI(false);
            //turn = NONE;
            if (cooldown > 0) cooldown--;
            else if (!cooldown) turn = NONE;
            /*
            if (RightBelow != DetectLane::null && LeftBelow != DetectLane::null)
                CarControl::preSteer = (RightBelow + LeftBelow) / 2;
            else if (RightBelow != DetectLane::null)
                CarControl::preSteer = RightBelow - Point(laneWidth / 2, 0);
            else if (LeftBelow != DetectLane::null)
                CarControl::preSteer = LeftBelow + Point(laneWidth / 2, 0);
            */
        }
    #elif TURN_METHOD == STICK_LANE
        is_steer_middle = false;
        if (vecLeftSpeed.dot(vecRightSpeed) < 0.99 ||
            vecLeftAngle.dot(vecRightAngle) < 0.99 ||
            vecLeftAngle.dot(vecLeftSpeed) < 0.99 ||
            vecRightAngle.dot(vecRightSpeed) < 0.99 ||
            (detect->getLeftLaneRaw().size() <= NORMAL_LANE_SIZE_LIM && detect->getRightLaneRaw().size() <= NORMAL_LANE_SIZE_LIM)){
            cout << vecLeftSpeed.dot(vecRightSpeed) << " " << vecLeftAngle.dot(vecRightAngle) << endl;
            cout << "Sap toi nga re" << null_count++ << endl;
            CarControl::preSpeed = TURN_SPEED;
            TRAFFIC_SIGN temp;
            temp = DetectSign::get_traffic_sign(true);
            //temp = TURN_RIGHT;
            //temp = TURN_LEFT;
            //temp = TURN_RIGHT;
            if (temp == ERROR) {
                cout << "Check haar path" << endl;
                return;
            }
            else if (temp != NONE) {
                turn = temp;
                cooldown = TRAFFIC_SIGN_COOLDOWN;
            }
            if (cooldown > 0 && turn != NONE) cooldown = TRAFFIC_SIGN_COOLDOWN;
            //else if (!cooldown) turn = NONE;
            if (turn == NONE) {
                cout << "Di thang" << endl;
                CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            }
            else if(turn == TURN_LEFT)  {
                cout << "Re trai" << endl;
                CarControl::preSteer = LeftBelowAngleA + Point(laneWidth / 2, 0);
            }
            else  {
                cout << "Re phai" << endl;
                CarControl::preSteer = RightBelowAngleA - Point(laneWidth / 2, 0);
            }
            //CarControl::maskROI = &maskRoiIntersection;
            setROI(true);
        }
        else {
            CarControl::preSteer = (LeftBelowAngleA + RightBelowAngleA) / 2;
            if (abs(errorAngle(CarControl::preSteer)) < SMALL_TURN_ANGLE_UPPER_LIM) CarControl::preSpeed = FAST_SPEED;
            else CarControl::preSpeed = SLOW_SPEED;
            setROI(false);
            //turn = NONE;
            if (cooldown > 0) cooldown--;
            else if (!cooldown) turn = NONE;
        }
    #endif
    //cout << turn << endl;
    //cout << CarControl::preSteer << endl;
    Mat test = Mat::zeros(Size(240,320), CV_8UC3);
    cv::line(test, RightAboveSpeedA, RightBelowSpeedA, Scalar(255,0,0), 2);
    cv::line(test, LeftAboveSpeedA, LeftBelowSpeedA, Scalar(0,255,0), 2);
    cv::line(test, RightAboveAngleA, RightBelowAngleA, Scalar(255,255,0), 2);
    cv::line(test, LeftAboveAngleA, LeftBelowAngleA, Scalar(0,255,255), 2);
    cv::imshow("Test", test);
    
    Obstacle vat_can = obstacle->get_obstacle(true);
    //cout << vat_can.is_there_obstacle() << endl;
    CarControl::preSteer = vat_can.offsetAngle(carPos, CarControl::preSteer, 30., 15., *detect, is_steer_middle);
    //vat_can.calc_warp(detect->getWarpMatrix());

    error = errorAngle(CarControl::preSteer);
    //cout << "sadad" << error << endl;
    angle.data = error;
    speed.data = CarControl::preSpeed;
    //cout << angle.data << endl;
    //cout << "Tur" << endl;
    //cout << error << endl;
    cv::circle(birdview_img, CarControl::preSteer, 3, Scalar(255,0,0), 3);
    cv::line(birdview_img, Point(120,300), CarControl::preSteer, Scalar(0,255,0), 1);
    cv::imshow("Center road", birdview_img);
    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
    return;
}

Point craftPoint(const Point& carPos, const float& angle){
    //x0 - carPos.x / carPos.y -y0 =tan(angle);
    float dy = 20;
    float dx = tan(angle) * dy;
    int x = carPos.x + dx;
    int y = carPos.y - dy;
    return Point(x, y);
}
Point craftPoint(const Point& carPos, const Vec2f& vec){
    //x0 - carPos.x / carPos.y -y0 =tan(angle);
    float dy = 20;
    float dx = vec[0] / -vec[1] * dy;
    int x = carPos.x + dx;
    int y = carPos.y - dy;
    return Point(x, y);
}
Point unWarpPoint(Point& pnt, Mat& warpMatrixInv){
    //cv::Matx33f M = cv::getPerspectiveTransform(srcQuad, dstQuad);
    Matx33f M = warpMatrixInv;
    cv::Point3f p(pnt.x, pnt.y, 1);
    p = M * p; // dst -> src
    // p = M * p; // src -> dst
    p = p * (1.0 / p.z);
    return cv::Point(p.x, p.y);
}
vector<Point> unWarpPoint(vector<Point>& pnts, Mat& warpMatrixInv){
    Matx33f M = warpMatrixInv;
    vector<Point> ret;
    for (const Point& pnt: pnts){
        cv::Point3f p(pnt.x, pnt.y, 1);
        p = M * p; // dst -> src
        // p = M * p; // src -> dst
        p = p * (1.0 / p.z);
        ret.push_back(Point(p.x, p.y));
    }
    return ret;
}