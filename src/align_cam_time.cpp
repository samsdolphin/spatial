#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "CustomMsg.h"

using namespace std;

ros::Publisher left_pub, right_pub;
ros::Subscriber left_sub, right_sub, livox_sub;

string write_path, filename, bag_path;
vector<double> livox_time;
size_t cnt = 1, livox_len;
bool left_ready = false, right_ready = false, livox_init = false;

void left_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if (cnt == livox_len)
    {
        ros::V_string v_nodes;
        ros::master::getNodes(v_nodes);

        std::string node_name = std::string("/my_record_node");
        auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name.c_str());
        if (it != v_nodes.end())
        {
            std::string cmd_str = "rosnode kill " + node_name;
            int ret = system(cmd_str.c_str());
            std::cout << "## stop rosbag record cmd: " << cmd_str << std::endl;
        }
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::ImagePtr img = cv_ptr->toImageMsg();
    img->header.stamp = ros::Time(livox_time[cnt]);

    left_pub.publish(img);
    left_ready = true;
    if (left_ready && right_ready)
    {
        cnt++;
        left_ready = false;
        right_ready = false;
    }
}

void right_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if (cnt == livox_len)
    {
        ros::V_string v_nodes;
        ros::master::getNodes(v_nodes);

        std::string node_name = std::string("/my_record_node");
        auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name.c_str());
        if (it != v_nodes.end())
        {
            std::string cmd_str = "rosnode kill " + node_name;
            int ret = system(cmd_str.c_str());
            std::cout << "## stop rosbag record cmd: " << cmd_str << std::endl;
        }
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::ImagePtr img = cv_ptr->toImageMsg();
    img->header.stamp = ros::Time(livox_time[cnt]);

    right_pub.publish(img);
    right_ready = true;
    if (left_ready && right_ready)
    {
        cnt++;
        left_ready = false;
        right_ready = false;
    }
}

void livox_callback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    if (!livox_init)
    {
        std::string path = "/home/sam/bag_name.bag";
        std::string topics = " /left/image /right/image /livox/imu /livox/lidar";
        std::string node_name = " __name:=my_record_node";
        std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + topics + node_name + "'";
        int ret = system(cmd_str.c_str());
        livox_init = true;
    }
}

void filter_lidar()
{
    rosbag::Bag bag_r, bag_w;
    bag_r.open(bag_path, rosbag::bagmode::Read);
    bag_w.open("/home/sam/lidar.bag", rosbag::bagmode::Write);
    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg"));
    rosbag::View view(bag_r, rosbag::TypeQuery(types));
    for (const rosbag::MessageInstance& m : view)
    {
        auto msg = *(m.instantiate<livox_ros_driver::CustomMsg>());
        // double secs = msg.header.stamp.sec;
        // double nsecs = msg.header.stamp.nsec * 1e-9;
        if (msg.header.stamp.toSec() > livox_time[0])
            bag_w.write("/livox/lidar", msg.header.stamp, msg);
    }
    cout<<"complete lidar"<<endl;
}

void filter_imu()
{
    rosbag::Bag bag_r, bag_w;
    bag_r.open(bag_path, rosbag::bagmode::Read);
    bag_w.open("/home/sam/imu.bag", rosbag::bagmode::Write);
    vector<string> types;
    types.push_back(string("/livox/imu"));
    rosbag::View view(bag_r, rosbag::TopicQuery(types));
    for (const rosbag::MessageInstance& m : view)
    {
        auto msg = *(m.instantiate<sensor_msgs::Imu>());
        if (msg.header.stamp.toSec() >= livox_time[1])
            bag_w.write("/livox/imu", msg.header.stamp, msg);
    }
    cout<<"complete imu"<<endl;
}

void attach_left_camera()
{
    rosbag::Bag bag_r, bag_w;
    bag_r.open(bag_path, rosbag::bagmode::Read);
    bag_w.open("/home/sam/left.bag", rosbag::bagmode::Write);
    vector<string> types;
    types.push_back(string("/left_camera/image"));
    rosbag::View view(bag_r, rosbag::TopicQuery(types));
    int n = 1;
    for (const rosbag::MessageInstance& m : view)
    {
        auto msg = *(m.instantiate<sensor_msgs::Image>());
        ros::Time t;
        t.sec = uint32_t(floor(livox_time[n]));
        t.nsec = uint32_t((livox_time[n] - floor(livox_time[n])) * 1e9);
        msg.header.stamp.sec = uint32_t(floor(livox_time[n]));
        msg.header.stamp.nsec = uint32_t((livox_time[n] - floor(livox_time[n])) * 1e9);
        if (n < livox_len) bag_w.write("/left_camera/image", t, msg);
        n++;
    }
    cout<<"complete left camera"<<endl;
}

void attach_right_camera()
{
    rosbag::Bag bag_r, bag_w;
    bag_r.open(bag_path, rosbag::bagmode::Read);
    bag_w.open("/home/sam/right.bag", rosbag::bagmode::Write);
    vector<string> types;
    types.push_back(string("/right_camera/image"));
    rosbag::View view(bag_r, rosbag::TopicQuery(types));
    int n = 1;
    for (const rosbag::MessageInstance& m : view)
    {
        auto msg = *(m.instantiate<sensor_msgs::Image>());
        ros::Time t;
        t.sec = uint32_t(floor(livox_time[n]));
        t.nsec = uint32_t((livox_time[n] - floor(livox_time[n])) * 1e9);
        msg.header.stamp.sec = uint32_t(floor(livox_time[n]));
        msg.header.stamp.nsec = uint32_t((livox_time[n] - floor(livox_time[n])) * 1e9);
        if (n < livox_len) bag_w.write("/right_camera/image", t, msg);
        n++;
    }
    cout<<"complete right camera"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_camera_time");
    ros::NodeHandle nh("~");

    nh.getParam("write_path", write_path);
    nh.getParam("bag_path", bag_path);
    
    // livox_sub = nh.subscribe("/livox/lidar", 10, livox_callback);
    // left_sub = nh.subscribe("/left_camera/image", 10, left_img_callback);
    // right_sub = nh.subscribe("/right_camera/image", 10, right_img_callback);

    left_pub = nh.advertise<sensor_msgs::Image>("/left/image", 10);
    right_pub = nh.advertise<sensor_msgs::Image>("/right/image", 10);
    
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
    livox_len = livox_time.size();
    cout<<"livox time length "<<livox_len<<endl;

    filter_lidar();
    filter_imu();
    attach_left_camera();
    attach_right_camera();

    ros::spin();
    return 0;
}