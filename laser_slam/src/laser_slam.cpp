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

#include "ground_segmentation/groundSegmentation.h"
#include <laser_slam/laser_slam.h>

// Clears the global map, integrates the points finding the transformation from
// the point cloud's acquisition frame to some global frame, and places
// everything into a single cloud.
void LaserSlam::regenerateGlobalMap() {
  ROS_INFO_STREAM("Laser SLAM::Regenerating Global Map");
  // clear the global map
  global_map->clearMap();

  // integrate the points
  PoseGraph *pose_graph = GSO->getPoseGraph();
  // Clears the global map, integrates the points.
  for (unsigned int i = 0; i < pose_graph->nodes.size(); i++) {
    // Transform keyframe into global frame
    Eigen::Affine3d trans;
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    tf::poseMsgToEigen(pose_graph->nodes[i].pose.pose, trans);
    pcl::transformPointCloud(*(pose_graph->nodes[i].keyframe), *trans_cloud,
                             trans);
    global_map->addToMap(trans_cloud, pose_graph->nodes[i].pose);
  }
}

bool LaserSlam::insertAndProcess(
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
    Eigen::Affine3d input_pose) {

  std::vector<Eigen::Affine3d> refined_pose_vec;

  double start_time = ros::Time::now().toSec();

  // insert the node into the graph
  Node gN;
  gN.prior_pose = input_pose;
  pose_graph = GSO->getPoseGraph();
  gN.idx = pose_graph->nodes.size();
  tf::poseEigenToMsg(input_pose, gN.pose.pose);
  *gN.keyframe = *input_cloud;
  GSO->addNode(gN);
  // Add the prior on the pose.
  GSO->addPosePrior(gN);

  // can't do anything unless we have more than 1 node
  if (pose_graph->nodes.size() < 2) {
    regenerateGlobalMap();
    return true;
  }

  for (int ref_iter = 0; ref_iter < this->num_refinement_iter; ref_iter++) {
    ROS_INFO_STREAM("Laser SLAM::Performing refinement iteration " << ref_iter);
    if (!ros::ok())
      break;

    // update all the node poses
    if (ref_iter > 0) {
      for (unsigned int j = 0; j < pose_graph->nodes.size(); j++) {
        Eigen::Affine3d trans;
        tf::poseEigenToMsg(refined_pose_vec[j], pose_graph->nodes[j].pose.pose);
      }
    }

    // Generate the pose graph to optimize.
    if (this->recompute_all_edges) { // If we want to find new edges using
                                     // k-nearest neighbours.

      // clear the edges
      pose_graph->edges.clear();

      // generate new keyframe edges
      for (unsigned int j = 0; j < pose_graph->nodes.size(); j++) {
        Node &current_node = pose_graph->nodes[j];
        vector<int> KFNN = scan_registration->getKNN(*pose_graph, current_node,
                                                     this->num_vertex_knn);

        for (unsigned int i = 0; i < KFNN.size(); i++) {
          double scale_div = 1;
          if (ref_iter > 0) {
            scale_div = (ref_iter + 1);
          }
          if (scan_registration->addEdgeToX(*pose_graph, KFNN[i],
                                            current_node.idx, scale_div)) {
            ROS_INFO_STREAM("Laser SLAM::Adding edge between scans "
                            << current_node.idx << " and " << (KFNN[i])
                            << " with scale division " << scale_div);
            GSO->addEdge(pose_graph->edges.back());
          }
        }
      }
    } else {
      // generate the edges for only the latest pose insertion instead of
      // globally.
      // Remove the edges that are connect to the last node.
      GSO->removeLastEdges();
      Node &current_node = pose_graph->nodes.back();
      vector<int> KFNN = scan_registration->getKNN(*pose_graph, current_node,
                                                   this->num_vertex_knn);
      int num_kfnn = KFNN.size();

      if (ref_iter > 0) {
        for (unsigned int i = 0; i < num_kfnn; i++) {
          ROS_INFO_STREAM("DEL:" << i);
          pose_graph->edges.pop_back();
        }
      }

      // Add the prior on the pose.
      GSO->addPosePrior(current_node);
      for (unsigned int i = 0; i < KFNN.size(); i++) {
        double scale_div = 1;
        if (ref_iter > 0) {
          scale_div = (ref_iter + 1);
        }
        if (scan_registration->addEdgeToX(*pose_graph, KFNN[i],
                                          current_node.idx, scale_div)) {
          ROS_INFO_STREAM("Laser SLAM::Adding edge between scans "
                          << current_node.idx << " and " << (KFNN[i])
                          << " with scale division " << scale_div);
          GSO->addEdge(pose_graph->edges.back());
        }
      }
    }

    // optimize Pose graph
    GSO->optimizeGraph();

    // Global Mapping Update
    regenerateGlobalMap();

    refined_pose_vec.clear();
    // ROS_INFO_STREAM("Saving Initial Poses for refinement");
    for (unsigned int j = 0; j < pose_graph->nodes.size(); j++) {
      Eigen::Affine3d trans;
      tf::poseMsgToEigen(pose_graph->nodes[j].pose.pose, trans);
      refined_pose_vec.push_back(trans);
    }
  }

  // Before:
  int end_pose = pose_graph->nodes.size() - 1;
  ROS_INFO_STREAM("Before Opt:");
  ROS_INFO_STREAM(pose_graph->nodes[end_pose].pose.pose.position.x);
  ROS_INFO_STREAM(pose_graph->nodes[end_pose].pose.pose.position.y);
  ROS_INFO_STREAM(pose_graph->nodes[end_pose].pose.pose.position.z);

  ROS_INFO_STREAM("After Opt:");
  ROS_INFO_STREAM(pose_graph->nodes[end_pose].pose.pose.position.x);
  ROS_INFO_STREAM(pose_graph->nodes[end_pose].pose.pose.position.y);
  ROS_INFO_STREAM(pose_graph->nodes[end_pose].pose.pose.position.z);

  ROS_INFO_STREAM("Laser SLAM::Done Insertion");
  double end_time = ros::Time::now().toSec();
  ROS_INFO_STREAM(
      "Laser SLAM::Total Insertion Time: " << (end_time - start_time) << "s");
  return true;
}

bool LaserSlam::insertPointCloud(laser_slam::InsertPointCloud::Request &req,
                                 laser_slam::InsertPointCloud::Response &res) {
  ROS_INFO("Laser SLAM::Processing new Point Cloud");
  // Convert to PCL point cloud.
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(req.point_cloud, *input_cloud);
  Eigen::Affine3d input_pose;
  tf::poseMsgToEigen(req.point_cloud_pose, input_pose);

  // Insert the point cloud into our global map.
  insertAndProcess(input_cloud, input_pose);

  // set response for drivability map
  res.insertion_complete = true;
  nav_msgs::OccupancyGrid::Ptr drivability_map;
  drivability_map = global_map->getDrivability();
  drivability_map->header.stamp = ros::Time::now();
  drivability_map->header.frame_id = "/global";
  res.drivability_map = *drivability_map;

  // set response for elevation map
  grid_map_msgs::GridMap elevation_msg;
  grid_map::GridMap elevation_map = global_map->getElevationGrid();
  elevation_map.setTimestamp(ros::Time::now().toNSec());
  elevation_map.setFrameId("/global");
  grid_map::GridMapRosConverter::toMessage(elevation_map, elevation_msg);
  res.elevation_map = elevation_msg;

  // set response for full map cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr full_map_cloud;
  full_map_cloud = global_map->getGlobalCloud();
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*full_map_cloud, cloud_msg);
  cloud_msg.header.frame_id = "/global";
  cloud_msg.header.stamp = ros::Time::now();
  res.full_map_cloud = cloud_msg;

  ROS_INFO("Laser SLAM::Finished Pointcloud insertion, Waiting for next point "
           "cloud");
  finished_insertion = true;
  return true;
}

int LaserSlam::run() {

  while (ros::ok()) {
    ros::spinOnce();

    if (finished_insertion) {

      // Publish poseGraph
      posegraph_pub.publish(getVisualMsg(pose_graph->nodes, pose_graph->edges));
      // Publish MLS
      global_map->visualize(mls_pub, "/global");
      finished_insertion = false;
    }
  }

  return 0;
}
