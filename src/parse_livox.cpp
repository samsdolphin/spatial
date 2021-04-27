#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "CustomMsg.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
typedef pcl::PointXYZINormal PointType;

using namespace std;
using namespace Eigen;

double secs_init, nsecs_init, last_time;
bool write_time, is_init = false;
string bag_path, write_path, filename;
vector<double> inc_time;

ros::Publisher pub_avia;

const bool time_list(PointType& x, PointType& y) {return (x.curvature < y.curvature);};

void parse_rosbag_time() 
{
    fstream file_;
    file_.open(bag_path, ios::in);
    if (!file_)
    {
        cout << "File " << bag_path << " does not exit" << endl;
        return;
    }
    
    rosbag::Bag bag;
    try
    {
        ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
        bag.open(bag_path, rosbag::bagmode::Read);
        ROS_INFO("Bag %s opened", bag_path.c_str());
    }
    catch (rosbag::BagException e)
    {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg")); 
    rosbag::View view(bag, rosbag::TypeQuery(types));

    ofstream file_w;
    filename = write_path + "inc_time";
    file_w.open(filename, std::ofstream::trunc);

    ofstream file;
    filename = write_path + "livox_time";
    file.open(filename, std::ofstream::trunc);
    
    for (const rosbag::MessageInstance& m : view)
    {
        auto msg = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
        double secs = msg.header.stamp.sec;
        double nsecs = msg.header.stamp.nsec;
        double inc, t;
        t = secs + nsecs * 1e-9;
        
        if (!is_init)
        {
            secs_init = secs;
            nsecs_init = nsecs;
            is_init = true;
            secs -= secs_init;
            nsecs -= nsecs_init;
            last_time = secs + nsecs * 1e-9;
            if (write_time) file << std::setprecision(9) << std::fixed << t;
        }
        else
        {
            secs -= secs_init;
            nsecs -= nsecs_init;
            inc = secs + nsecs * 1e-9 - last_time;
            inc_time.push_back(inc);
            last_time = secs + nsecs * 1e-9;
            file_w << inc * 1e3 << "\n";
            if (write_time) file << "\n" << std::setprecision(9) << std::fixed << t;
        }
    }
    file.close();
    file_w.close();
    cout<<"complete"<<endl;
    plt::named_plot("livox_time",inc_time);
    plt::legend();
    plt::show();
}

void avia_callback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    // int timebase_sec = floor(msg->timebase*1e-9);
    // int timebase_nsec = (msg->timebase*1e-9 - timebase_sec)*1e9;

    uint32_t msg_sec = msg->header.stamp.sec;

    size_t pt_size = msg->point_num;
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
    pc->points.reserve(pt_size);
    for (size_t i = 0; i < pt_size; i++)
    {
        if((msg->points[i].line < 1) && ((msg->points[i].tag & 0x30) == 0x10))
        {
            pc->points[i].x = msg->points[i].x;
            pc->points[i].y = msg->points[i].y;
            pc->points[i].z = msg->points[i].z;
            pc->points[i].intensity = msg->points[i].reflectivity;
            pc->points[i].curvature = msg->points[i].offset_time;
        }
    }
    std::sort(pc->points.begin(), pc->points.end(), time_list);

    // cout<<"nsec "<<std::setprecision(9) << std::fixed << msg->header.stamp.nsec<<endl;
    sensor_msgs::PointCloud2 avia_msg;
    size_t part = pt_size / 10;
    bool jump = false;
    for (size_t i = 0; i < 10; i++)
    {
        size_t st = i * part;
        size_t ed = (i+1) * part;
        pcl::PointCloud<PointType>::Ptr pc_(new pcl::PointCloud<PointType>);
        pc_->reserve(part);
        for (size_t j = st; j < ed; j++)
        {
            pc_->points[j-st].x = pc->points[j].x;
            pc_->points[j-st].y = pc->points[j].y;
            pc_->points[j-st].z = pc->points[j].z;
            pc_->points[j-st].intensity = pc->points[j].intensity;
            pc_->points[j-st].curvature = pc->points[j].curvature; // curvature is used as time
        }
        uint32_t msg_nsec = uint32_t(pc->points[st].curvature) + msg->header.stamp.nsec;
        if (msg_nsec > 1e9 && !jump)
        {
            msg_nsec -= 1e9;
            msg_sec += 1;
            jump = true;
        }
        else if (msg_nsec > 1e9)
            msg_nsec -= 1e9;
        cout<<msg_sec<<"."<<msg_nsec<<endl;
        // cout<<"offset_time "<<int(pc->points[st].curvature)+msg->header.stamp.nsec<<endl;
        
        pcl::toROSMsg(*pc, avia_msg);
        avia_msg.header.frame_id = "livox_frame";
        // avia_msg.header.stamp = cur_t;
        pub_avia.publish(avia_msg);
    } 
    
    // string filename = data_path + to_string(cnt) + ".dat";
    // cout<<"writing "<<cnt<<endl;
    // cnt++;

    // ofstream outFile(filename, ios::out | ios::binary);
    // for (size_t i = 0; i < pt_size; i++)
    // {
    //     if (pc->points[i].x > 0)
    //     {
    //         mypcl::mypoint p(pc->points[i].x, pc->points[i].y, pc->points[i].z);
    //         outFile.write((char*)&p, sizeof(p));
    //     }
    // }
    // outFile.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parse_livox");
    ros::NodeHandle nh("~");
    
    nh.getParam("bag_path", bag_path);
    nh.getParam("write_path", write_path);
    nh.getParam("write_time", write_time);

    // ros::Subscriber sub_avia = nh.subscribe("/livox/lidar", 1000, avia_callback);
    // pub_avia = nh.advertise<sensor_msgs::PointCloud2>("/avia/lidar", 100);

    parse_rosbag_time();
    
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}