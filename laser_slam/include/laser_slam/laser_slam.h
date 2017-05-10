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
#ifndef LASER_SLAM_H
#define LASER_SLAM_H

#include <ros/ros.h>
#include <iostream>
#include <map>
#include <vector>
//ros/pcl headers
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
//our libraries
#include <mls/mls.h>
//local headers
#include <laser_slam/global_mapping_viz.h>
#include <laser_slam/scan_registration.h>
#include <laser_slam/csv_tools.h>
 //service
#include <laser_slam/InsertPointCloud.h>
//backend
#include <laser_slam/graph_slam_types.h>
#include <laser_slam/optimizer_base.h>
#include <laser_slam/optimizer_impl_g2o.h>



class LaserSlam{
public:
//constructor
LaserSlam(int x_size, int y_size, double map_res, double vehicle_h, double init_leaf_size, double icp_distance_max, double trans_thresh, double rot_thresh): x_cell_size(x_size), y_cell_size(y_size), map_resolution(map_res),vehicle_height(vehicle_h) 
 {
 	//initialize the global map
 	global_map = new MLS(x_cell_size,y_cell_size,map_resolution,false,vehicle_height);
 	//initialize the scan registration
 	scan_registration = new ScanRegistration(init_leaf_size, icp_distance_max,trans_thresh,rot_thresh);
 	//setup 
    graphVizInit();
    scan_registration->setup();
    GSO->initOptimizer();
    finished_insertion = false;
    ROS_INFO("Laser SLAM::Waiting for first pointcloud");
 }
 //public functions
 bool insertPointCloud(laser_slam::InsertPointCloud::Request  &req, laser_slam::InsertPointCloud::Response &res);
 int run();

 //accessors
 void setNumRefinementIterations(int ref_iter ) {num_refinement_iter = ref_iter;}
 int getNumRefinementIterations() {return num_refinement_iter;}
 void setNumVertexKNN(int vertex_knn ) {num_vertex_knn = vertex_knn;}
 int getNumVertexKNN() {return num_vertex_knn;}
 void setRecomputeAllEdges(bool all_edges){recompute_all_edges = all_edges;}

//ROS I/O
ros::Publisher  posegraph_pub;
ros::Publisher  mls_pub;
ros::ServiceServer service;

//MLS map
MLS* global_map; 

private:

//functions
void regenerateGlobalMap();
bool insertAndProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, Eigen::Affine3d input_pose);


//variables

int x_cell_size;
int y_cell_size;
double map_resolution;
double vehicle_height;


//tuning parameters
int num_refinement_iter = 3;
int num_vertex_knn = 3;
bool recompute_all_edges = false;

GraphSlamOptimizerG2O opt_;
GraphSlamOptimizer * GSO = &opt_;
PoseGraph* pose_graph;
ScanRegistration* scan_registration;
bool finished_insertion;

};

#endif
