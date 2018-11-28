/*
g++ opencv_get_from_camera.cpp -I/usr/include/opencv2 -L/usr/local/lib -lopencv_highgui -lopencv_core

https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=videocapture#videocapture
*/

#include <iostream>

#include "opencv2/opencv.hpp"

int main(int argc, char const *argv[])
{
    cv::VideoCapture cap(0);
    if(!cap.isOpened())  // check if we succeeded
    {
        std::cout << "could not open camera 0" << std::endl;
        return -1;
    } else {
        std::cout << "DID open camera 0" << std::endl;
    }
    cv::Mat edges;
    cv::namedWindow("edges",1);

    for(int ii_=0;ii_<1;) {
        cv::Mat frame;
        cap >> frame;
        cv::cvtColor(frame,edges, CV_BGR2GRAY);
    }


    /* code */
    return 0;
}
