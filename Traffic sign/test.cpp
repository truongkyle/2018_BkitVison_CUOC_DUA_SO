#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace std;
using namespace cv;
void detectAndDisplay( Mat frame );
CascadeClassifier traffic_sign_cascade;
int main( int argc, const char** argv )
{
    //-- 1. Load the cascades
    if(!traffic_sign_cascade.load("./cascade_right.xml"))
    {
        cout << "--(!)Error loading face cascade\n";
        return -1;
    };
    
    while (true)
    {
        Mat frame = cv::imread("./IMG_723.jpg");
        if( frame.empty() )
        {
            cout << "--(!) No captured frame -- Break!\n";
            break;
        }
        //-- 3. Apply the classifier to the frame
        detectAndDisplay(frame);
        if( waitKey(10) == 27 )
        {
            break; // escape
        }
    }
    return 0;
}
void detectAndDisplay( Mat frame )
{
    std::vector<Rect> traffic_region;
    traffic_sign_cascade.detectMultiScale(frame, traffic_region);
    for ( size_t i = 0; i < traffic_region.size(); i++ )
    {
            cv::rectangle(frame, traffic_region[i], Scalar(0,255,0), 2);
    }
    //-- Show what you got
    imshow( "Capture - Face detection", frame );
}
