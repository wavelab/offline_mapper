/************************************************************************
 *
 *
 *  Copyright 2015  Arun Das (University of Waterloo)
 *                      [adas@uwaterloo.ca]
 *                  James Servos (University of Waterloo)
 *                      [jdservos@uwaterloo.ca]
 *
 *
 *************************************************************************/

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <vector>

// ros/pcl headers
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <laser_slam/InsertPointCloud.h>
#include <laser_slam/csv_tools.h>

using namespace std;

int main(int argc, char **argv) {
  int num_scans = 10;
  // Initialize ROS
  ros::init(argc, argv, "laser_slam_service_tester");
  ros::NodeHandle nh("~");

  // ROS PUBLISHERS
  ros::Publisher map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/laserslam/full_pointcloud", 1, true);
  ros::Publisher drivablility_pub = nh.advertise<nav_msgs::OccupancyGrid>(
      "/laserslam/drivability_map", 1, true);
  ros::Publisher elevation_pub =
      nh.advertise<grid_map_msgs::GridMap>("/laserslam/elevation_map", 1, true);
  ros::ServiceClient client = nh.serviceClient<laser_slam::InsertPointCloud>(
      "/laser_slam/InsertPointCloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;

  std::string scan_file_path;
  std::string pose_file_path;
  // get the scan and pose file paths
  if (nh.getParam("/scan_file_path", scan_file_path)) {
    ROS_INFO_STREAM("using scan_file_path: " << scan_file_path);
  } else {
    ROS_ERROR("scan_file_path parameter not set");
    return -1;
  }
  if (nh.getParam("/pose_file_path", pose_file_path)) {
    ROS_INFO_STREAM("using pose_file_path: " << pose_file_path);
  } else {
    ROS_ERROR("pose_file_path parameter not set");
    return -1;
  }

  ros::Time start = ros::Time::now();

  for (int i = 0; i < num_scans; i++) {

    if (!ros::ok())
      break;

    int num = 1 + i;

    std::string in_file_scan = scan_file_path;
    std::string in_file_pose = pose_file_path;
    char numStrScan[15];
    char numStrPose[15];
    sprintf(numStrScan, "%04d.ply", num);
    sprintf(numStrPose, "%04d.csv", num);
    in_file_scan += std::string(numStrScan);
    in_file_pose += std::string(numStrPose);

    input_cloud = load_scan_ply(in_file_scan.c_str());
    ROS_INFO_STREAM("input cloud_size is: " << input_cloud->size());

    Eigen::Affine3d pose_trans = Eigen::Affine3d::Identity();
    pose_trans = load_pose(in_file_pose.c_str());
    ROS_INFO_STREAM("pose data is: " << endl << pose_trans.matrix());

    // call the service for insertion
    laser_slam::InsertPointCloud srv;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*input_cloud, cloud_msg);
    srv.request.point_cloud = cloud_msg;

    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(pose_trans, pose_msg);
    srv.request.point_cloud_pose = pose_msg;
    if (client.call(srv)) {
      ROS_INFO_STREAM("Publishing Responses...");
      elevation_pub.publish(srv.response.elevation_map);
      map_cloud_pub.publish(srv.response.full_map_cloud);
      drivablility_pub.publish(srv.response.drivability_map);
      ROS_INFO_STREAM("Published Responses");

    } else {
      ROS_ERROR("Failed to call service");
      return 1;
    }

    ros::Duration(3).sleep();
  }

  ROS_INFO_STREAM("All Scans Complete");

  ROS_WARN_STREAM("Execution time: " << (ros::Time::now() - start));
  return 0;
}
