#include "detectlane.h"
#include "carcontrol.h"
#include "detect_traffic_sign.h"
#define DIST_SPEED 15
#define DIST_ANGLE 3
#define LIM_DIST_SPEED 5
#define LIM_DIST_ANGLE 3

int cooldown = 0;
Mat* CarControl::maskROI;
Mat CarControl::maskRoiLane;
Mat CarControl::maskRoiIntersection;
Point CarControl::preSteer;
float CarControl::preSpeed;
Vec2f CarControl::vecLeftSpeed, CarControl::vecRightSpeed, CarControl::vecLeftAngle, CarControl::vecRightAngle;
Point CarControl::LeftAboveSpeedA, CarControl::RightAboveSpeedA, CarControl::LeftBelowSpeedA, CarControl::RightBelowSpeedA;
Point CarControl::LeftAboveAngleA, CarControl::RightAboveAngleA, CarControl::LeftBelowAngleA, CarControl::RightBelowAngleA;
TRAFFIC_SIGN turn = NONE;
void CarControl::init(){
    vector<Point> list_point_ROI = {Point(0,240), Point(0,150), Point(100,80), Point(220,80), Point(320,150), Point(320,240)};
    vector<Point> list_point_noROI = {Point(0,240), Point(0,80), Point(320,80), Point(320,240)};
    maskRoiLane = Mat::zeros(Size(320,240), CV_8UC3);
    maskRoiIntersection = Mat::zeros(Size(320,240), CV_8UC3);
    cv::fillConvexPoly(maskRoiLane, list_point_ROI, Scalar(255,255,255));
    cv::fillConvexPoly(maskRoiIntersection, list_point_noROI, Scalar(255,255,255));
    maskROI = &maskRoiIntersection;
}

CarControl::CarControl(){
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("Team1_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("Team1_speed",10);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst){
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = CV_PI;
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
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
    int i = left.size() - 8;
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
                k = temp - DIST_SPEED;
                while (k >= max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    if (left[k] != DetectLane::null) {
                        LeftAboveSpeed = left[k];
                        break;
                    }
                    k--;
                }
                k = temp - DIST_ANGLE;
                while (k >= max(temp - DIST_ANGLE - LIM_DIST_ANGLE, 0)){
                    if (left[k] != DetectLane::null) {
                        LeftAboveAngle = left[k];
                        break;
                    }
                    k--;
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
                k = temp - DIST_SPEED;
                while (k >= max(temp - DIST_SPEED - LIM_DIST_SPEED, 0)){
                    if (right[k] != DetectLane::null) {
                        RightAboveSpeed = right[k];
                        break;
                    }
                    k--;
                }
                k = temp - DIST_ANGLE;
                while (k >= max(temp - DIST_ANGLE - LIM_DIST_ANGLE, 0)){
                    if (right[k] != DetectLane::null) {
                        RightAboveAngle = right[k];
                        break;
                    }
                    k--;
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
        if (LeftAboveSpeed != DetectLane::null) LeftAboveSpeedA = LeftAboveSpeed;
        if (LeftBelowSpeed != DetectLane::null) LeftBelowSpeedA = LeftBelowSpeed;
        if (LeftAboveAngle != DetectLane::null) LeftAboveAngleA = LeftAboveAngle;
        if (LeftBelowAngle != DetectLane::null) LeftBelowAngleA = LeftBelowAngle;
        if (RightAboveSpeed != DetectLane::null) RightAboveSpeedA = RightAboveSpeed;
        if (RightBelowSpeed != DetectLane::null) RightBelowSpeedA = RightBelowSpeed;
        if (RightAboveAngle != DetectLane::null) RightAboveAngleA = RightAboveAngle;
        if (RightBelowAngle != DetectLane::null) RightBelowAngleA = RightBelowAngle;
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
    vecLeftSpeed = Vec2f(LeftAboveSpeedA.x - LeftBelowSpeedA.x, LeftAboveSpeedA.y - LeftBelowSpeedA.y);
    vecLeftSpeed = vecLeftSpeed / cv::norm(vecLeftSpeed);
    vecRightSpeed = Vec2f(RightAboveSpeedA.x - RightBelowSpeedA.x, RightAboveSpeedA.y - RightBelowSpeedA.y);
    vecRightSpeed = vecRightSpeed / cv::norm(vecRightSpeed);
    vecLeftAngle = Vec2f(LeftAboveAngleA.x - LeftBelowAngleA.x, LeftAboveAngleA.y - LeftBelowAngleA.y);
    vecLeftAngle = vecLeftAngle / cv::norm(vecLeftAngle);
    vecRightAngle = Vec2f(RightAboveAngleA.x - RightBelowAngleA.x, RightAboveAngleA.y - RightBelowAngleA.y);
    vecRightAngle = vecRightAngle / cv::norm(vecRightAngle);
    if (vecLeftSpeed.dot(vecRightSpeed) < 0.99 || 
        vecLeftAngle.dot(vecRightAngle) < 0.98 ||
        vecLeftAngle.dot(vecLeftSpeed) < 0.98 ||
        vecRightAngle.dot(vecRightSpeed) < 0.98){
        cout << vecLeftSpeed.dot(vecRightSpeed) << " " << vecLeftAngle.dot(vecRightAngle) << endl;
        cout << "Sap toi nga re" << null_count++ << endl;
        CarControl::preSpeed = 40;
        TRAFFIC_SIGN temp;
        temp = DetectSign::get_traffic_sign(true);
        if (temp == ERROR) {
            cout << "Check haar path" << endl;
            return;
        }
        if (temp != NONE) {
            turn = temp;
            cooldown = 5;
        }
        else if (cooldown > 0 && turn != NONE) cooldown = 5;
        //else if (!cooldown) turn = NONE;
        if (turn == NONE) {
            cout << "Di thang" << endl;
            CarControl::preSteer = (LeftBelowSpeedA + RightBelowSpeedA) / 2;
        }
        else if(turn == TURN_LEFT)  {
            cout << "Re trai" << endl;
            CarControl::preSteer = LeftBelowAngleA + Point(laneWidth / 2, 0);
        }
        else  {
            cout << "Re phai" << endl;
            CarControl::preSteer = RightBelowAngleA - Point(laneWidth / 2, 0);
        }
        CarControl::maskROI = &maskRoiIntersection;
    }
    else {
        CarControl::preSteer = (LeftBelowSpeedA + RightBelowSpeedA) / 2;
        CarControl::preSpeed = 60;
        CarControl::maskROI = &maskRoiLane;
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
    //cout << turn << endl;
    Mat test = Mat::zeros(Size(240,320), CV_8UC3);
    cv::line(test, RightAboveSpeedA, RightBelowSpeedA, Scalar(255,0,0), 2);
    cv::line(test, LeftAboveSpeedA, LeftBelowSpeedA, Scalar(0,255,0), 2);
    cv::line(test, RightAboveAngleA, RightBelowAngleA, Scalar(255,255,0), 2);
    cv::line(test, LeftAboveAngleA, LeftBelowAngleA, Scalar(0,255,255), 2);
    cv::imshow("Test", test);
    error = errorAngle(CarControl::preSteer);
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