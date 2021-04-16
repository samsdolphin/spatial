#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > vector_quad;
typedef pcl::PointXYZRGBNormal PointType;

namespace mypcl
{
    class mypoint
    {
    public:
        float _x, _y, _z;
        mypoint(float x = 0.0, float y = 0.0 , float z = 0.0) : _x(x), _y(y), _z(z){}
    };

    struct pose
    {
        pose(Eigen::Quaterniond _q, Eigen::Vector3d _t) : q(_q), t(_t){}
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
    };

    pcl::PointCloud<PointType> read_colorpoint(std::string path)
    {
        pcl::PointCloud<PointType> pc;
        pc.points.resize(2e6);
        std::fstream file;
        file.open(path);
        size_t cnt = 0;
        double x, y, z;
        double r, g, b;
        while (!file.eof())
        {
            file >> x >> y >> z >> r >> g >> b;
            pc.points[cnt].x = x;
            pc.points[cnt].y = y;
            pc.points[cnt].z = z;
            pc.points[cnt].r = uint8_t(r);
            pc.points[cnt].g = uint8_t(g);
            pc.points[cnt].b = uint8_t(b);
            cnt++;
            if (cnt == 2e6)
                break;
        }
        cnt--;
        file.close();
        pc.points.resize(cnt);
        return pc;
    }

    pcl::PointCloud<PointType> read_pointcloud(std::string path)
    {
        pcl::PointCloud<PointType> pc;
        pc.points.resize(5e6);
        std::fstream file;
        file.open(path);
        size_t cnt = 0;
        double x, y, z;
        while (!file.eof())
        {
            file >> x >> y >> z;
            if (x > 0)
            {
                pc.points[cnt].x = x;
                pc.points[cnt].y = y;
                pc.points[cnt].z = z;
                cnt++;
            }
        }
        cnt--;
        file.close();
        pc.points.resize(cnt);
        return pc;
    }

    pcl::PointCloud<PointType> read_pointdat(std::string filename)
    {
        mypoint p;
        std::ifstream inFile(filename, std::ios::in | std::ios::binary);
        pcl::PointCloud<PointType> pc;
        pc.points.resize(5e6);
        size_t cnt = 0;
        while(inFile.read((char *)&p, sizeof(p)))
        {
            // if (p._x > 0)
            {
                pc.points[cnt].x = p._x;
                pc.points[cnt].y = p._y;
                pc.points[cnt].z = p._z;
                cnt++;
            }
        }
        inFile.close();
        pc.points.resize(cnt);
        return pc;
    }

    std::vector<pose> read_pose(std::string path)
    {
        std::vector<pose> pose_vec;
        std::fstream file;
        file.open(path);
        double tx, ty, tz, w, x, y, z;
        while (!file.eof())
        {
            file >> tx >> ty >> tz >> w >> x >> y >> z;
            pose_vec.push_back(pose(Eigen::Quaterniond(w, x, y, z),
                                    Eigen::Vector3d(tx, ty, tz)));
        }
        file.close();
        return pose_vec;
    }

    void transform_pointcloud(pcl::PointCloud<PointType> const& pc_in,
                              pcl::PointCloud<PointType>& pt_out,
                              Eigen::Vector3d t,
                              Eigen::Quaterniond q)
    {
        size_t size = pc_in.points.size();
        pt_out.points.resize(size);
        for (size_t i = 0; i < size; i++)
        {
            Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
            Eigen::Vector3d pt_to;
            pt_to = q * pt_cur + t;
            pt_out.points[i].x = pt_to.x();
            pt_out.points[i].y = pt_to.y();
            pt_out.points[i].z = pt_to.z();
            pt_out.points[i].r = pc_in.points[i].r;
            pt_out.points[i].g = pc_in.points[i].g;
            pt_out.points[i].b = pc_in.points[i].b;
        }
    }

    pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
                                                 pcl::PointCloud<PointType> pc2)
    {
        size_t size1 = pc1->points.size();
        size_t size2 = pc2.points.size();
        pc1->points.resize(size1 + size2);
        for (size_t i = size1; i < size1 + size2; i++)
        {
            pc1->points[i].x = pc2.points[i-size1].x;
            pc1->points[i].y = pc2.points[i-size1].y;
            pc1->points[i].z = pc2.points[i-size1].z;
            pc1->points[i].r = pc2.points[i-size1].r;
            pc1->points[i].g = pc2.points[i-size1].g;
            pc1->points[i].b = pc2.points[i-size1].b;
            // pc1->points[i].intensity = pc2.points[i-size1].intensity;
        }
        return pc1;
    }

    double compute_inlier_ratio(std::vector<double> residuals, double ratio)
    {
        std::set<double> dis_vec;
        for (size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
            dis_vec.insert(fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) + fabs(residuals[3 * i + 2]));

        return *(std::next(dis_vec.begin(), (int)((ratio) * dis_vec.size())));
    }

    void write_pose(std::vector<pose> pose_vec, std::string path)
    {
        std::ofstream file;
        file.open(path + "pose.json", std::ofstream::trunc);
        Eigen::Quaterniond q0(pose_vec[0].q.w(),
                                pose_vec[0].q.x(), 
                                pose_vec[0].q.y(),
                                pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0),
                            pose_vec[0].t(1),
                            pose_vec[0].t(2));
        for (size_t i = 0; i < pose_vec.size(); i++)
        {
            Eigen::Vector3d t(0, 0, 0);
            Eigen::Quaterniond q(1, 0, 0, 0);
            t << q0.inverse()*(pose_vec[i].t-t0);
            q = q0.inverse()*pose_vec[i].q;
            if (i == pose_vec.size()-1)
                file << t(0) << " " << t(1) << " " << t(2) << " "
                    << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z();
            else
                file << t(0) << " " << t(1) << " " << t(2) << " "
                    << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();
    }

    void write_ref(std::vector<pose> ref_vec, std::string path)
    {
        std::ofstream file;
        file.open(path + "ref.json", std::ofstream::trunc);
        for (size_t i = 0; i < ref_vec.size(); i++)
        {
            Eigen::Quaterniond q = ref_vec[i].q;
            Eigen::Vector3d t = ref_vec[i].t;
            if (i == ref_vec.size()-1)
                file << t(0) << " " << t(1) << " " << t(2) << " "
                    << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z();
            else
                file << t(0) << " " << t(1) << " " << t(2) << " "
                    << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();
    }

    void write_pose(std::vector<pose> pose_vec, std::vector<pose> ref_vec, std::string path)
    {
        std::ofstream file;
        file.open(path + "pose.json", std::ofstream::trunc);
        Eigen::Quaterniond q0(pose_vec[0].q.w(),
                                pose_vec[0].q.x(), 
                                pose_vec[0].q.y(),
                                pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0),
                            pose_vec[0].t(1),
                            pose_vec[0].t(2));
        for (size_t i = 0; i < pose_vec.size(); i++)
        {
            Eigen::Vector3d t(0, 0, 0);
            Eigen::Quaterniond q(1, 0, 0, 0);
            t << q0.inverse()*(pose_vec[i].t-t0);
            q = q0.inverse()*pose_vec[i].q;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();

        file.open(path + "ref.json", std::ofstream::trunc);
        for (size_t i = 0; i < ref_vec.size() - 1; i++)
        {
            Eigen::Quaterniond q = ref_vec[i].q;
            Eigen::Vector3d t = ref_vec[i].t;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file.close();
    }

    void log_pose(std::vector<pose> pose_vec, std::string path, int plane = 0,
                  double threshold = 0.0, double calib = 0.0)
    {
        std::ofstream file;
        file.open(path + "pose_log.json", std::ofstream::app);
        Eigen::Quaterniond q0(pose_vec[0].q.w(),
                                pose_vec[0].q.x(), 
                                pose_vec[0].q.y(),
                                pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0),
                            pose_vec[0].t(1),
                            pose_vec[0].t(2));
        file << plane << " " << threshold << " " << calib << "\n";
        for (size_t i = 0; i < pose_vec.size() - 1; i++)
        {
            Eigen::Vector3d t(0, 0, 0);
            Eigen::Quaterniond q(1, 0, 0, 0);
            t << q0.inverse()*(pose_vec[i].t-t0);
            q = q0.inverse()*pose_vec[i].q;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file << "\n";
        file.close();
    }

    void log_ref(std::vector<pose> ref_vec, std::string path, int plane = 0,
                 double threshold = 0.0, double calib = 0.0)
    {
        std::ofstream file;
        file.open(path + "ref_log.json", std::ofstream::app);
        file << plane << " " << threshold << " " << calib << "\n";
        for (size_t i = 0; i < ref_vec.size() - 1; i++)
        {
            Eigen::Quaterniond q = ref_vec[i].q;
            Eigen::Vector3d t = ref_vec[i].t;
            file << t(0) << " " << t(1) << " " << t(2) << " "
                 << q.w() << " "<< q.x() << " "<< q.y() << " "<< q.z() << "\n";
        }
        file << "\n";
        file.close();
    }

    // void make_arrow_func(pcl::PointCloud<PointType> &pl, visualization_msgs::MarkerArray &marker_array, int &mid, ros::Time ct)
    // {  
    //     for(uint i=0; i<pl.size(); i++)
    //     {    
    //         visualization_msgs::Marker marker;    
    //         marker.header.frame_id = "camera_init";
    //         marker.header.stamp = ct;  
    //         marker.id = mid; 
    //         mid++;  
    //         marker.action = visualization_msgs::Marker::ADD;   
    //         marker.type = visualization_msgs::Marker::ARROW;  
    //         marker.color.a = 1; 
    //         marker.color.r = 1;  
    //         marker.color.g = 0;  
    //         marker.color.b = 0;  
    //         marker.scale.x = 0.02;   
    //         marker.scale.y = 0.05;  
    //         marker.scale.z = 0.1; 
    //         marker.lifetime = ros::Duration();  
    //         geometry_msgs::Point apoint;  
    //         apoint.x = pl[i].x;   
    //         apoint.y = pl[i].y;  
    //         apoint.z = pl[i].z;  
    //         marker.points.push_back(apoint);  
    //         apoint.x += pl[i].normal_x; 
    //         apoint.y += pl[i].normal_y; 
    //         apoint.z += pl[i].normal_z;
    //         marker.points.push_back(apoint);   
    //         marker_array.markers.push_back(marker);  
    //     }
    // }
}

#endif