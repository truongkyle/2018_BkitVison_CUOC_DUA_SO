#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"
#include "detect_traffic_sign.h"

#define VIDEO_PATH "/home/hoquangnam/Documents/CuocDuaSo/outcpp.avi"
#define IMAGE_PATH "/home/hoquangnam/Documents/CuocDuaSo/Lane_image/IMG_735.jpg"
#define VIDEO_OR_IMAGE "video" // Or "image"
#define HAAR_TRAFFIC_SIGN_LEFT_DIR "/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/Traffic sign/cascade_left.xml"
#define HAAR_TRAFFIC_SIGN_RIGHT_DIR "/home/hoquangnam/Documents/CuocDuaSo/test_streaming_ros/Traffic sign/cascade_right.xml"
#define STREAM true

//int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
//int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
VideoCapture capture(VIDEO_PATH);
//VideoWrir video("/home/hoquangnam/Documents/CuocDuaSo/outcpp.avi",CV_FOURCC('M','J','P','G'),25, Size(320,240));
DetectLane *detect;
CarControl *car;
int skipFrame = 1;
void detectAndDisplay( Mat frame );
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static int count = 0;
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        DetectSign::store_image(cv_ptr->image);
        //waitKey(1);
        detect->update(cv_ptr->image);
        car->driverCar(detect);
        cv::imshow("View", cv_ptr->image);
        //std::string path = "/home/hoquangnam/Documents/CuocDuaSo/Ảnh fastest/Test" + std::to_string(count) + ".jpg";
        //cv::imwrite(path, cv_ptr->image);
        //video.write(cv_ptr->image);
        //count++;
        //cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        //cout << "Test";
        capture >> src;
        if (src.empty()) break;
        //cout << "test" << endl;
        //DetectSign::get_traffic_sign(src, true);
        DetectSign::store_image(src);
        detect->update(src);
        car->driverCar(detect);
        imshow("View", src);
        waitKey(30);
    }
}
void imageProcess(){
    Mat src = cv::imread(IMAGE_PATH);
    while(true){
        if (src.empty()) return;
        //DetectSign::get_traffic_sign(src, true);
        DetectSign::store_image(src);
        detect->update(src);
        car->driverCar(detect);
        cv::imshow("View", src);       //cv::imshow("Test", CarControl::maskROI);
        waitKey(30);
        //detectAndDisplay(src);
        //waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    //cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
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
    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

        ros::spin();
    } 
    else {
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
void detectAndDisplay( Mat frame )
{
    CascadeClassifier traffic_sign_cascade;
    if(!traffic_sign_cascade.load("/home/hoquangnam/Documents/CuocDuaSo/Traffic sign/cascade_right.xml"))
    {
        cout << "--(!)Error loading face cascade\n";
        return;
    };
    std::vector<Rect> traffic_region;
    traffic_sign_cascade.detectMultiScale(frame, traffic_region);
    for ( size_t i = 0; i < traffic_region.size(); i++ )
    {
            cv::rectangle(frame, traffic_region[i], Scalar(0,255,0), 2);
    }
    //-- Show what you got
    imshow( "Capture - Face detection", frame );
}
