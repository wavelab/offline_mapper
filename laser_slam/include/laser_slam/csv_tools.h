#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <fstream>

//opens a csv file and loads it into a pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr  load_scan_ply(const char* file_name);
pcl::PointCloud<pcl::PointXYZ>::Ptr  load_scan(const char* file_name);
Eigen::Affine3d  load_pose(const char* file_name);
