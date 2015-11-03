#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>

using namespace std;


//number of threads
#define WORKER_DIM 2

//canny Threshold
#define THRESHOLD 50

void rgb2gray(cv::Mat rgb , cv::Mat& gray);
void edge(cv::Mat gray , cv::Mat& edged);
void create_workers();
void* canny(void* p);

int counters = 0;
cv::Mat gray;
cv::Mat edged;

int main(int argc, char **argv){
    std::string nodeName = "edge_detector_node";
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(nodeName, 1);

    //choose an image
    cv::Mat frame = cv::imread("/home/mohammad/Pictures/Wallpapers/xavi-messi.jpg");

    ros::Rate loop_rate(100);

    while (nh.ok()) {

        cv::cvtColor(frame,gray,CV_RGB2GRAY);
        rgb2gray(frame,gray);
        edged = gray;

        // create some workers for parallelism
        create_workers();
        cv::Mat image;
        cv::cvtColor(edged,image,CV_GRAY2BGR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void* canny(void* args){
    int *points_for_begin = (int *)args;
    int cols = (gray.cols)/WORKER_DIM;
    int rows = (gray.rows)/WORKER_DIM;
    int xBegin = points_for_begin[0]*cols;
    int yBegin = points_for_begin[1]*rows;
    int xEnd = (xBegin + cols)-1;
    int yEnd = (yBegin + rows)-1;

    for(int x = xBegin ; x < xEnd ; x++){
        for(int y = yBegin ; y < yEnd ; y++){
            int counter = 0;
            if(abs(gray.at<uchar>(y,x) - gray.at<uchar>(y,x+1)) > THRESHOLD){
                counter++;
            }
            if(abs(gray.at<uchar>(y,x) - gray.at<uchar>(y+1,x)) > THRESHOLD){
                counter++;
            }
            if(abs(gray.at<uchar>(y,x) - gray.at<uchar>(y+1,x+1)) > THRESHOLD){
                counter++;
            }
            if(counter > 1){
                edged.at<uchar>(y,x) = 255;
            }
            else{
                edged.at<uchar>(y,x) = 0;
            }
        }
    }
    return points_for_begin;
}
void create_workers(){
    pthread_t worker[WORKER_DIM][WORKER_DIM];
    pthread_attr_t attr;
    for(int j = 0 ; j < WORKER_DIM ; j++){
        for(int i = 0 ; i < WORKER_DIM ; i++){
            int *args = new int[2];
            args[0] = i;
            args[1] = j;
            pthread_attr_init(&attr);
            pthread_create(&worker[i][j],&attr,canny,args);
        }
    }
    for(int j = 0 ; j < WORKER_DIM ; j++){
        for(int i = 0 ; i < WORKER_DIM ; i++){
            pthread_join(worker[i][j], NULL);
        }
    }
}

void rgb2gray(cv::Mat rgb , cv::Mat& gray){
    for(int x = 0 ; x < rgb.cols ; x++){
        for(int y = 0 ; y < rgb.rows ; y++){
            int r = rgb.at<cv::Vec3b>(y,x)[0];
            int g = rgb.at<cv::Vec3b>(y,x)[1];
            int b = rgb.at<cv::Vec3b>(y,x)[2];
            int grayValue = 0.299*r + 0.587*g + 0.114*b ;
            gray.at<uchar>(y,x) = grayValue;       
        }
    }
}
