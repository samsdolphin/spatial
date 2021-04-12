#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

string data_path, filename;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "plane_extractor");
    ros::NodeHandle nh("~");
    ros::Publisher pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/map_surf", 100);
    ros::Publisher pub_debug = nh.advertise<sensor_msgs::PointCloud2>("/debug_surf", 100);
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseArray>("/poseArrayTopic", 10);

    nh.getParam("data_path", data_path);

    vector<double> x, y, z, roll, pitch, yaw;
    double in;
    std::fstream file;
    filename = data_path + "log/process_state/x.txt";
    file.open(filename);
    while (!file.eof())
    {
        file >> in;
        x.push_back(in);
    }
    file.close();

    filename = data_path + "log/process_state/y.txt";
    file.open(filename);
    while (!file.eof())
    {
        file >> in;
        y.push_back(in);
    }
    file.close();

    filename = data_path + "log/process_state/z.txt";
    file.open(filename);
    while (!file.eof())
    {
        file >> in;
        z.push_back(in);
    }
    file.close();

    filename = data_path + "log/process_state/roll.txt";
    file.open(filename);
    while (!file.eof())
    {
        file >> in;
        roll.push_back(in);
    }
    file.close();

    filename = data_path + "log/process_state/pitch.txt";
    file.open(filename);
    while (!file.eof())
    {
        file >> in;
        pitch.push_back(in);
    }
    file.close();

    filename = data_path + "log/process_state/yaw.txt";
    file.open(filename);
    while (!file.eof())
    {
        file >> in;
        yaw.push_back(in);
    }
    file.close();

    Matrix3d R_, R, Rx, Ry, Rz;
    double a = 0, b = 0, g = 0;
    R_ << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    double pi = 3.1415926;
    
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
    
    ros::Time cur_t = ros::Time::now();
    geometry_msgs::PoseArray parray;
	parray.header.frame_id = "camera_init";
    parray.header.stamp = cur_t;

    for (size_t i = 0; i < x.size(); i++)
    {
        cout<<"frame "<<i;
        
        a = pi / 180 * (roll[i]);
        b = pi / 180 * (-pitch[i]);
        g = pi / 180 * (450-yaw[i]);

        cout<<" roll "<<a*180/pi;
        cout<<" pitch "<<b*180/pi;
        cout<<" yaw "<<g*180/pi<<endl;
        
        Rx << 1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a);
        Ry << cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);
        Rz << cos(g), -sin(g), 0, sin(g), cos(g), 0, 0, 0, 1;
        R = Rz * Ry * Rx;
        Quaterniond q(R);
        Vector3d t(x[i], y[i], z[i]);
        *pc = mypcl::read_pointdat(data_path+to_string(i)+".dat");
        // mypcl::transform_pointcloud(*pc, *pc, Vector3d(0, 0, 0), Quaterniond(R_));
        mypcl::transform_pointcloud(*pc, *pc, t, q);
        pc_surf = mypcl::append_cloud(pc_surf, *pc);

        geometry_msgs::Pose apose;
        apose.orientation.w = q.w();
        apose.orientation.x = q.x();
        apose.orientation.y = q.y();
        apose.orientation.z = q.z();
        apose.position.x = t(0);
        apose.position.y = t(1);
        apose.position.z = t(2);
        parray.poses.push_back(apose);
        
        pub_pose.publish(parray);

        sensor_msgs::PointCloud2 debugMsg;
        pcl::toROSMsg(*pc, debugMsg);
        debugMsg.header.frame_id = "camera_init";
        debugMsg.header.stamp = cur_t;
        pub_debug.publish(debugMsg);

        pub_surf.publish(debugMsg);
    }
    // pub_pose.publish(parray);

    // sensor_msgs::PointCloud2 debugMsg;
    // pcl::toROSMsg(*pc_surf, debugMsg);
    // debugMsg.header.frame_id = "camera_init";
    // debugMsg.header.stamp = cur_t;
    // pub_surf.publish(debugMsg);

	// ros::Rate loop_rate(1);
    // while (ros::ok())
    // {
    //     // pub_surf.publish(debugMsg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}