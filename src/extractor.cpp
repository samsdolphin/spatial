#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

// typedef pcl::PointXYZI PointType;
string data_path;
int cnt = 0;

void surf_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *pc);
    string filename = data_path + to_string(cnt) + ".dat";
    cnt++;

    ofstream outFile(filename, ios::out | ios::binary);
    for (size_t i = 0; i < pc->points.size(); i++)
        if (pc->points[i].x > 0)
        {
            mypcl::mypoint p(pc->points[i].x, pc->points[i].y, pc->points[i].z);
            outFile.write((char*)&p, sizeof(p));
        }
    outFile.close();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "plane_extractor");
    ros::NodeHandle nh("~");
    ros::Subscriber sub_surf =
        nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 10000, surf_callback);

    nh.getParam("data_path", data_path);

	ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}