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

#include <laser_slam/laser_slam.h>

template <typename TYPE>
void LoadParameter(std::string param_name, TYPE &param_var,
                   ros::NodeHandle &nh) {
  if (nh.getParam(param_name, param_var)) {
    ROS_INFO_STREAM("Laser SLAM:: " << param_name << ": " << param_var);
  } else {
    ROS_ERROR_STREAM("Laser SLAM:: could not load parameter " << param_name);
    exit(-1);
  }
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "laser_slam");
  ros::NodeHandle nh("~"); // use ~ to access node's namespace

  // params to be set for the algorithm
  int map_x_size_meters;
  int map_y_size_meters;
  double map_resolution;
  double mls_vehicle_height;
  int scanreg_num_vertex_knn;
  int scanreg_num_ref_iter;
  double scanreg_init_leaf_size;
  double scanreg_init_icp_distance_max;
  double scanreg_icp_trans_thresh;
  double scanreg_icp_rot_thresh;
  double mls_thickness_threshold;
  double mls_height_threshold;
  double mls_cluster_sigma_factor;
  double mls_cluster_dist_threshold;
  double mls_cluster_combine_dist;
  bool scanreg_recompute_all_edges;

  ROS_INFO_STREAM("Loading parameters from server...");

  // try to get all the params
  LoadParameter<int>("map_x_size_meter", map_x_size_meters, nh);
  LoadParameter<int>("map_y_size_meter", map_y_size_meters, nh);
  LoadParameter<double>("map_resolution", map_resolution, nh);
  LoadParameter<double>("mls_vehicle_height", mls_vehicle_height, nh);
  LoadParameter<int>("scanreg_num_vertex_knn", scanreg_num_vertex_knn, nh);
  LoadParameter<int>("scanreg_num_refinement_iterations", scanreg_num_ref_iter,
                     nh);
  LoadParameter<double>("scanreg_init_leaf_size", scanreg_init_leaf_size, nh);
  LoadParameter<double>("scanreg_init_icp_distance_max",
                        scanreg_init_icp_distance_max, nh);
  LoadParameter<double>("scanreg_icp_trans_thresh", scanreg_icp_trans_thresh,
                        nh);
  LoadParameter<double>("scanreg_icp_rot_thresh", scanreg_icp_rot_thresh, nh);
  LoadParameter<double>("mls_thickness_threshold", mls_thickness_threshold, nh);
  LoadParameter<double>("mls_height_threshold", mls_height_threshold, nh);
  LoadParameter<double>("mls_cluster_sigma_factor", mls_cluster_sigma_factor,
                        nh);
  LoadParameter<double>("mls_cluster_dist_threshold",
                        mls_cluster_dist_threshold, nh);
  LoadParameter<double>("mls_cluster_combine_dist", mls_cluster_combine_dist,
                        nh);
  LoadParameter<bool>("scanreg_recompute_all_edges",
                      scanreg_recompute_all_edges, nh);

  ROS_INFO_STREAM("Finished loading parameters");

  int map_x_cells = (int)(map_x_size_meters / map_resolution);
  int map_y_cells = (int)(map_y_size_meters / map_resolution);

  LaserSlam laser_slam(map_x_cells, map_y_cells, map_resolution,
                       mls_vehicle_height, scanreg_init_leaf_size,
                       scanreg_init_icp_distance_max, scanreg_icp_trans_thresh,
                       scanreg_icp_rot_thresh);

  // set up all the tuning parameters
  laser_slam.setNumVertexKNN(scanreg_num_vertex_knn);
  laser_slam.setNumRefinementIterations(scanreg_num_ref_iter);
  laser_slam.global_map->setThicknessTheshold(mls_thickness_threshold);
  laser_slam.global_map->setHeightTheshold(mls_height_threshold);
  laser_slam.global_map->setClusterSigmaFactor(mls_cluster_sigma_factor);
  laser_slam.global_map->setClusterDistTheshold(mls_cluster_dist_threshold);
  laser_slam.global_map->setClusterCombineDist(mls_cluster_combine_dist);
  laser_slam.setRecomputeAllEdges(scanreg_recompute_all_edges);

  laser_slam.posegraph_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "/laserslam/pose_graph", 1, true);
  laser_slam.mls_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/laserslam/mls", 1, true);
  laser_slam.service = nh.advertiseService(
      "InsertPointCloud", &LaserSlam::insertPointCloud, &laser_slam);

  // run the main SLAM thread
  laser_slam.run();

  return 0;
}
