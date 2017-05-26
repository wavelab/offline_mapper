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
#ifndef MLS_H
#define MLS_H

#include "ground_segmentation/groundSegmentation.h"
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <stack>
#include <vector>

// Defines (should NEVER change)
//   parameters that will change should be parameters with 'setXXX' functions
//#define SINGULAR_THRESHOLD  0.001

class Cluster {
private:
public:
  Eigen::Vector3d mean;
  Eigen::Matrix3d cov;
  double num_pts;

  Cluster() {
    mean = Eigen::Vector3d::Zero();
    cov = Eigen::Matrix3d::Zero();
    num_pts = 0;
  }
};

// Grid cell class
class Cell {
private:
public:
  std::vector<Cluster> clusters;
  std::deque<pcl::PointXYZ> cloud; // temp storage of cell points

  int drivable; // is cell drivable [1 = yes, 0 = no, -1 = unknown
  bool updated; // flag for cell update

  Cell() {
    drivable = -1;
    updated = false;
  }
};

class Grid {
private:
  Cell *data;

public:
  int size_x;
  int size_y;

  int origin_x;
  int origin_y;

  Grid() {}
  Grid(int size_x_, int size_y_) {
    size_x = size_x_;
    size_y = size_y_;
    origin_x = 0;
    origin_y = 0;
    data = new Cell[size_x_ * size_y_];
  }
  ~Grid() { delete[] data; }

  Cell *operator()(int x, int y) {
    int ix = (x + origin_x);
    int iy = (y + origin_y);
    if (ix < 0)
      ix += size_x;
    else if (ix >= size_x)
      ix -= size_x;
    if (iy < 0)
      iy += size_y;
    else if (iy >= size_y)
      iy -= size_y;

    return data + ix + size_x * iy;
  }

  void shiftOrigin(int dx, int dy) {
    origin_x = (origin_x + dx);
    origin_y = (origin_y + dy);

    if (origin_x < 0)
      origin_x += size_x;
    else if (origin_x >= size_x)
      origin_x -= size_x;

    if (origin_y < 0)
      origin_y += size_y;
    else if (origin_y >= size_y)
      origin_y -= size_y;
  }
};

// MLS class
class MLS {
private:
  // MAP
  Grid grid;
  int size_x;
  int size_y;
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud;
  nav_msgs::OccupancyGrid::Ptr drivabilityGrid;
  grid_map::GridMap elevationGrid;

  // Map Parameters
  double resolution;
  bool rolling;
  int update_dist;  //'distance' to update [in cells]
  double max_range; // max range of measurement to use
  int max_clusters;
  int max_cluster_points;
  int min_cluster_points;

  // Drivability parameters
  double thickness_threshold;    // cluster thickness threshold
  double height_threshold;       // height diff [m] between cells
  double cluster_sigma_factor;   // Gassian std dev to correspond a point to a
                                 // cluster
  double cluster_dist_threshold; // additional delta offset from the
                                 // sigma_factor
  double cluster_combine_dist;   // at what distance do we combine two clusters?
  double robot_height;           // distance from velodyne to ground

  // Used for partial updates and rolling map
  geometry_msgs::PoseStamped curPose;

  // flags
  bool disable_pointcloud;

  // for occupancy grid mode
  groundSegmentation gSeg;
  double occupancy_increment;
  double occupancy_decrement;

  void updateCell(int x, int y);

public:
  MLS() {
    std::cout << std::endl << "YOU DID NOT INITALIZE THE MLS!!!" << std::endl;
  }
  MLS(int size_x_, int size_y_, double res, bool roll, double robot_size = 1.45)
      : grid(size_x_, size_y_),
        global_cloud(new pcl::PointCloud<pcl::PointXYZI>),
        obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        ground_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        drivabilityGrid(new nav_msgs::OccupancyGrid) {
    resolution = res;
    rolling = roll;
    size_x = size_x_;
    size_y = size_y_;
    max_range = 1e10;
    update_dist = fmin((int)max_range / resolution, size_x / 2);
    max_clusters = 50;
    max_cluster_points = 200;
    min_cluster_points = 10;

    drivabilityGrid->header.frame_id = "/global";
    drivabilityGrid->info.resolution = res;
    drivabilityGrid->info.width = size_x;
    drivabilityGrid->info.height = size_y;
    geometry_msgs::Pose offset;
    offset.position.x = -(resolution * size_x / 2);
    offset.position.y = -(resolution * size_y / 2);
    drivabilityGrid->info.origin = offset;
    drivabilityGrid->data.resize(size_x * size_y);

    // setup elevation grid
    elevationGrid.add("elevation", 0);
    double size_x_meters = size_x * res;
    double size_y_meters = size_y * res;
    elevationGrid.setGeometry(grid_map::Length(size_x_meters, size_y_meters),
                              res);

    thickness_threshold = 0.03; // default covariance threshold
    height_threshold = 0.3;     // default threshold
    cluster_sigma_factor = 3;
    cluster_dist_threshold = 0.5;
    cluster_combine_dist = 0.2;
    // drive_dist_threshold = 1.0;
    robot_height = robot_size;

    disable_pointcloud = false;

    occupancy_increment = 1;
    occupancy_decrement = 0.3;

    // initialize the starting location in the MLS
    if (!rolling) {
      int set_size = 1.0 / resolution; // set 1m radius around robot
      Cluster cluster;
      cluster.mean[2] = -robot_size;
      cluster.num_pts = min_cluster_points; // FIXME magic number
      cluster.cov(2, 2) = 0.01;             // FIXME magic number
      for (int i = -set_size; i <= set_size; i++) {
        for (int j = -set_size; j <= set_size; j++) {
          Cell *cell = grid(i + size_x / 2, j + size_y / 2);
          cluster.mean[0] = i * resolution;
          cluster.mean[1] = j * resolution;
          cell->clusters.push_back(cluster);
        }
      }
    }
  }

  void clearMap();
  void addToMap(pcl::PointCloud<pcl::PointXYZI>::Ptr
                    input_cloud); // assumed pose already set
  void addToMap(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                geometry_msgs::PoseStamped pose); // sets pose first
  void addToOccupancy(
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud); // sets pose first
  void setPose(geometry_msgs::PoseStamped pose);
  void offsetMap(const geometry_msgs::PoseStamped &pose);

  void updateElevationGrid();

  pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalCloud() { return global_cloud; }
  nav_msgs::OccupancyGrid::Ptr getDrivability() { return drivabilityGrid; }
  grid_map::GridMap getElevationGrid();
  void getSegmentedClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &ground);

  void filterPointCloud(double xy, double z);

  // setters
  void setThicknessTheshold(double thickness_thresh) {
    thickness_threshold = thickness_thresh;
  }
  void setHeightTheshold(double height_thresh) {
    height_threshold = height_thresh;
  }

  void setClusterDistTheshold(double cluster_dist_thresh) {
    cluster_dist_threshold = cluster_dist_thresh;
  }
  void setClusterCombineDist(double cluster_combine) {
    cluster_combine_dist = cluster_combine;
  }
  void setClusterSigmaFactor(double cluster_sigma) {
    cluster_sigma_factor = cluster_sigma;
  }
  void setRobotHeight(double robot_h) { robot_height = robot_h; }

  void setMaxClusters(double clusters) { max_clusters = clusters; }
  void setMaxClusterPoints(double cluster_points) {
    max_cluster_points = cluster_points;
  }
  void setMinClusterPoints(double cluster_points) {
    min_cluster_points = cluster_points;
  }

  void setMaxRange(double range) { max_range = range; }
  void setUpdateDistMeters(double updist) {
    update_dist = (int)(updist / resolution);
  }
  void setDisablePointCloud(bool disable) {
    disable_pointcloud = disable;
  } // increases performance

  // visualize the MLS clusters
  // pub needs to be a MarkerArray publisher
  void visualize(ros::Publisher pub, std::string frame = "/global");
};

#endif
