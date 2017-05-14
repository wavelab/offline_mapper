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
#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

struct Edge {
  int from;
  int to;
  geometry_msgs::PoseStamped edge;
  Eigen::MatrixXd edgeCov;
  Eigen::MatrixXd edgeInf;
  int ctype;

  Edge() : edgeCov(6, 6) {}
};

// Node of a graph containing a pose and a point cloud.
struct Node {
  int idx;                                      // The index of the node.
  geometry_msgs::PoseStamped pose;              // The pose.
  pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe; // The point cloud.
  Node() : keyframe(new pcl::PointCloud<pcl::PointXYZ>) {}
  Eigen::Affine3d prior_pose; // Prior pose.
  Eigen::MatrixXd
      prior_information; // Prior information matrix. (Doesn't seem to be used.)
};

typedef std::vector<struct Edge> EdgeList;
typedef std::vector<struct Node> NodeList;

// Basic pose graph container storing nodes and edges in a list.
struct PoseGraph {
  NodeList nodes;
  EdgeList edges;
};

#endif
