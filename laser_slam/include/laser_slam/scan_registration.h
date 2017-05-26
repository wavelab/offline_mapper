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
#ifndef SCANREGISTRATION_H
#define SCANREGISTRATION_H

#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <laser_slam/graph_slam_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <vector>

namespace Eigen {
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class ScanRegistration {
public:
  ScanRegistration(double init_leaf_size, double icp_distance_max, double trans,
                   double rot)
      : course_scale_initial_leaf_size(init_leaf_size),
        course_scale_icp_distance_max(init_leaf_size),
        icp_trans_threshold(trans), icp_rot_threshold(rot) {}
  void setup();
  bool addEdgeToX(PoseGraph &pG, int from, int to, double scale_div);
  std::vector<int> getKNN(PoseGraph &pG, Node &gN, unsigned int K);

private:
  double getNearestKF(PoseGraph &pG, Node &gN);
  bool calcEdgeIcp(Edge &gE, PoseGraph &pG, double scale_div);
  Eigen::Matrix6d
  computeEdgeInformationLUM(pcl::PointCloud<pcl::PointXYZI>::Ptr &source_trans,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &target,
                            double max_corr_dist);
  // variables
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> registration_icp;
  double course_scale_initial_leaf_size;
  ;
  double course_scale_icp_distance_max;
  double icp_trans_threshold;
  double icp_rot_threshold;
};

#endif
