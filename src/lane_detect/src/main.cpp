#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"
#include "detect_traffic_sign.h"
#include "polyfit.h"
#define VIDEO_PATH "/home/hoquangnam/Documents/CuocDuaSo/outcpp.avi"
//#define VIDEO_PATH "/home/hoquangnam/Documents/CuocDuaSo/outcpp.avi"
#define IMAGE_PATH "/home/hoquangnam/Documents/CuocDuaSo/Lane_image/IMG_0659.jpg"
#define VIDEO_OR_IMAGE "video" // Or "image"
#define HAAR_TRAFFIC_SIGN_LEFT_DIR "/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/Traffic sign/cascade_left_2.xml"
#define HAAR_TRAFFIC_SIGN_RIGHT_DIR "/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/Traffic sign/cascade_right_2.xml"
#define STREAM true
//cv::polyfit();
//int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
//int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
VideoCapture capture(VIDEO_PATH);
//VideoWriter video("/home/hoquangnam/Documents/CuocDuaSo/outcpp2.avi",CV_FOURCC('M','J','P','G'),25, Size(320,240));
DetectLane *detect;
CarControl *car;
int skipFrame = 1;
void detectAndDisplay(Mat frame);
void draw_polygon(Mat& dst, const vector<Point>);
void imageCallback(const sensor_msgs::ImageConstPtr &msg){
    static int count = 0;
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat& src = cv_ptr->image;
        DetectSign::store_image(src);
        //waitKey(1);
        detect->update(src);
        car->driverCar(detect);
        draw_polygon(src, CarControl::get_list_point_ROI());
        cv::circle(src, unWarpPoint(car->get_center_road(), detect->getWarpMatrixInv()), 1, Scalar(0,0,255), 2);
        cv::imshow("View", src);

        //std::string path = "/home/hoquangnam/Documents/CuocDuaSo/Ảnh fastest/Test" + std::to_string(count) + ".jpg";
        //cv::imwrite(path, cv_ptr->image);
        //video.write(cv_ptr->image);
        //count++;
        //cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess(){
    Mat src;
    while (true){
        //cout << "Test";
        capture >> src;
        if (src.empty())
            break;
        //cout << "test" << endl;
        //DetectSign::get_traffic_sign(src, true);
        DetectSign::store_image(src);
        detect->update(src);
        car->driverCar(detect);
        draw_polygon(src, CarControl::get_list_point_ROI());
        cv::circle(src, unWarpPoint(car->get_center_road(), detect->getWarpMatrixInv()), 1, Scalar(0,0,255), 2);
        imshow("View", src);
        //waitKey(0);
        waitKey(0);
    }
}
void imageProcess(){
    while (true){
        Mat src = cv::imread(IMAGE_PATH);
        if (src.empty())
            return;
        //DetectSign::get_traffic_sign(src, true);
        DetectSign::store_image(src);
        detect->update(src);
        car->driverCar(detect);
        draw_polygon(src, CarControl::get_list_point_ROI());
        cv::circle(src, unWarpPoint(car->get_center_road(), detect->getWarpMatrixInv()), 1, Scalar(0,0,255), 2);
        cv::imshow("View", src); //cv::imshow("Test", CarControl::maskROI);
        waitKey(30);
        //detectAndDisplay(src);
        //waitKey(30);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    //cv::namedWindow("BinaryRoad");
    //cv::namedWindow("ThresholdRoad");
    cv::namedWindow("ThresholdLane");
    //cv::namedWindow("Bird View Road");
    cv::namedWindow("Bird View Lane");
    //cv::namedWindow("Bird View Filled");
    //cv::namedWindow("Bird View fix shadow");
    //cv::namedWindow("Point");
    cv::namedWindow("Lane Detect");
    //cv::namedWindow("Hough Lines");
    //cv::namedWindow("Canny");
    cv::namedWindow("Center road");
    cv::namedWindow("Test");
    CarControl::init();
    DetectSign::init(HAAR_TRAFFIC_SIGN_LEFT_DIR, HAAR_TRAFFIC_SIGN_RIGHT_DIR);
    detect = new DetectLane();
    car = new CarControl();
    detect->init();
    if (STREAM){
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("team207_image", 1, imageCallback);

        ros::spin();
    }
    else{
        //cv::startWindowThread();
        //cout << "Test" << endl;
        if (!strcmp(VIDEO_OR_IMAGE, "video"))
            videoProcess();
        else if (!strcmp(VIDEO_OR_IMAGE, "image"))
            imageProcess();
    }
    //video.release();
    cv::destroyAllWindows();
}
/*
void detectAndDisplay(Mat frame)
{
    CascadeClassifier traffic_sign_cascade;
    if (!traffic_sign_cascade.load("/home/hoquangnam/Documents/CuocDuaSo/Traffic sign/cascade_right.xml"))
    {
        cout << "--(!)Error loading face cascade\n";
        return;
    };
    std::vector<Rect> traffic_region;
    traffic_sign_cascade.detectMultiScale(frame, traffic_region);
    for (size_t i = 0; i < traffic_region.size(); i++)
    {
        cv::rectangle(frame, traffic_region[i], Scalar(0, 255, 0), 2);
    }
    //-- Show what you got
    imshow("Capture - Face detection", frame);
}
*/
/*
int main(int argc, char **argv)
{

    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    int start_s = clock(); // clock start
//0    1.3963    2.7925    4.1888    5.5851    6.9813    8.3776    9.7738   11.1701   12.5664
//0    0.9848    0.3420   -0.8660   -0.6428    0.6428    0.8660   -0.3420   -0.9848   -0.0000
    //vector<Point> points{Point(1,1), Point(1.2,1), Point(3,3), Point(7,4), Point(7,5)};
    vector<Point2f> points{Point2f(0,0), Point2f(1.3963,0.9848), Point2f(2.7925,0.3420), Point2f(4.1888,-0.8660), Point2f(5.5851,-0.6428), Point2f(6.9813,0.6428)};
    // polyFit function used to get coeffecients
    vector<float> ret = fitPoly(points, 4);
    for (float f: ret)
        cout << f << " ";
    cout << endl;
    // clock stopped to get execution time
    int stop_s = clock();
    // execution time in milliseconds
    cout << "time taken to execute this program is : " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 <<" millisecond"<< endl;
    return 0;
}
*/
void draw_polygon(Mat& dst, const vector<Point> list_point){
    for (int i = 0; i < list_point.size(); i++)
        cv::line(dst, list_point[i], list_point[(i+1) % list_point.size()], Scalar(0,255,0), 3);
}
//void 