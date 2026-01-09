#include <Eigen/Core>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <chrono>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle nh;
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/dense_viewer/pointcloud", 1);
    ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>("/dense_viewer/trajectory", 1);

    std::map<double, Eigen::Isometry3d> estimation_traj;
    std::string pose_path = "/home/da/project/ros/orb_dvl2_ws/src/dvl2/dvl2_results/dense/WholeTank_Medium_traj.txt";
    if (nh.getParam("/dense_ros/pose_path", pose_path))
    {
        ROS_INFO("Got pose_path: %s", pose_path.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param '/dense_ros/pose_path'");
    }
    std::ifstream estimation_file(pose_path);
    if (!estimation_file.is_open()) {
        std::cout << "Failed to open file:"<<pose_path << std::endl;
        return -1;
    }
    std::string line;
    while (std::getline(estimation_file, line)) {
        std::istringstream iss(line);
        if (line[0] == '#') {
            continue;
        }
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            std::cout << "Failed to parse line: " << line << std::endl;
            continue;
        }

        Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        Eigen::Matrix3d R = quaternion.normalized().toRotationMatrix();
        Eigen::Vector3d t(tx, ty, tz);
        Eigen::Isometry3d T;
        T.setIdentity();
        T.pretranslate(t);
        T.rotate(R);
        estimation_traj.insert(std::make_pair(timestamp, T));

    }

    estimation_file.close();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pcd_file = "";
    if (nh.getParam("/dense_ros/pcd_path", pcd_file))
    {
        ROS_INFO("Got pose_path: %s", pcd_file.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param '/dense_ros/pcd_path'");
    }
    pcl::io::loadPCDFile(pcd_file,*global_map);
    //oupt point cloud info
    ROS_INFO_STREAM("Loaded "
                    << global_map->width * global_map->height
                    << " data points from pcd with the following fields: "
                    << pcl::getFieldsList(*global_map));

    pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
    vgf.setInputCloud(global_map);
    vgf.setLeafSize(0.01, 0.01, 0.01);
    vgf.filter(*global_map);

    Eigen::Isometry3d T_rviz_c = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R_r_c;
    //assign value to R_r_c
    R_r_c << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
    T_rviz_c.rotate(R_r_c);
    T_rviz_c.pretranslate(Eigen::Vector3d(0, 0, 0));
    //tranform pointcloud
    // pcl::transformPointCloud(*global_map, *global_map, T_rviz_c.matrix());

    nav_msgs::Path traj_msg;
    traj_msg.header.frame_id = "map";
    traj_msg.header.stamp = ros::Time::now();

    for(auto it:estimation_traj){
       Eigen::Isometry3d T_c0_cj = it.second;
        // Eigen::Isometry3d T_c0_cj = T_rviz_c * it.second;

        Eigen::Vector3d t = T_c0_cj.translation();
        Eigen::Quaterniond q(T_c0_cj.rotation());
        geometry_msgs::PoseStamped p;
        p.header = traj_msg.header;
        p.pose.position.x = t.x();
        p.pose.position.y = t.y();
        p.pose.position.z = t.z();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();

        traj_msg.poses.push_back(p);
    }

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*global_map, pointcloud_msg);



    // set ros spin to publish at 10 hz
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        pointcloud_msg.header.frame_id = "map";
        pointcloud_msg.header.stamp = ros::Time::now();
        pointcloud_pub.publish(pointcloud_msg);
        traj_msg.header.frame_id = "map";
        traj_msg.header.stamp = ros::Time::now();
        trajectory_pub.publish(traj_msg);
        ROS_INFO_STREAM("pub map and traj");
//        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
