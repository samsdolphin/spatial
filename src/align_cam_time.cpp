#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

cv::Mat imgCallback;
ros::Publisher image_pub;

string write_path, filename;
vector<double> livox_time;
int cnt = 0;

static void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::ImagePtr img = cv_ptr->toImageMsg();
    img->header.stamp = ros::Time(livox_time[cnt]);
    cout<<std::setprecision(9) << std::fixed<<livox_time[cnt]<<endl;
    image_pub.publish(img);
    cnt++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_camera_time");
    ros::NodeHandle nh("~");

    nh.getParam("write_path", write_path);
    
    ros::Subscriber image_sub;
    image_sub = nh.subscribe("/left_camera/image", 10, ImageCallback);
    image_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 10);
    
    fstream file;
    filename = write_path + "livox_time";
    file.open(filename);
    double t;
    while (!file.eof())
    {
        file >> t;
        livox_time.push_back(t);
    }
    file.close();
    cout<<"livox time length "<<livox_time.size()<<endl;

    ros::spin();
    return 0;
}