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

#include <cmath>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <vector>
// ros/pcl headers
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <anm_msgs/VehicleState.h>
#include <eigen_conversions/eigen_msg.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/NavSatFix.h>

#include <laser_slam/InsertPointCloud.h>
#include <laser_slam/csv_tools.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

double get_2d_dist(geometry_msgs::Pose &pt1, geometry_msgs::Pose &pt2) {
  double dx, dy;
  dx = pt1.position.x - pt2.position.x;
  dy = pt1.position.y - pt2.position.y;
  return sqrt(dx * dx + dy * dy);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "laser_slam_service_from_bag");
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

  std::string bag_file_path;
  std::string velo_topic;
  std::string ekf_topic;
  std::string gps_topic;
  std::vector<std::string> topics;
  double dis_inc, alt_ref;
  int skip;
  int counter = 0;

  geometry_msgs::Pose last_pose, pose_msg;
  geometry_msgs::PoseStamped velo_pose;

  bool register_next = true;
  bool initialized = false;
  if (nh.getParam("/skip_scans", skip)) {
    ROS_INFO_STREAM("skipping scans to start " << skip);
  } else {
    skip = 0;
    ROS_ERROR("not skipping scans");
    return -1;
  }

  // get the bag file path
  if (nh.getParam("/bag_file_path", bag_file_path)) {
    ROS_INFO_STREAM("using bag_file_path: " << bag_file_path);
  } else {
    ROS_ERROR("bag_file_path parameter not set");
    return -1;
  }

  if (nh.getParam("/velodyne_topic_name", velo_topic)) {
    ROS_INFO_STREAM("using pointcloud2 topic: " << velo_topic);
    topics.push_back(velo_topic);
  } else {
    ROS_ERROR("velodyne_topic_name parameter not set");
    return -1;
  }

  if (nh.getParam("/ekf_topic_name", ekf_topic)) {
    ROS_INFO_STREAM("using eft topic name: " << ekf_topic);
    topics.push_back(ekf_topic);
  } else {
    ROS_ERROR("ekf_topic_name parameter not set");
    return -1;
  }

  if (nh.getParam("/distance_increment", dis_inc)) {
    ROS_INFO_STREAM("using distance increment: " << dis_inc);
  } else {
    ROS_ERROR("distance_increment parameter not set");
    return -1;
  }

  if (nh.getParam("/gps_topic_name", gps_topic)) {
    ROS_INFO_STREAM("using gps_topic_name: " << gps_topic);
    topics.push_back(gps_topic);
  } else {
    ROS_ERROR("gps_topic_name parameter not set");
    return -1;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_file_path, rosbag::bagmode::Read); // throws exception if fails
    ROS_INFO_STREAM("Bag is open");
  } catch (rosbag::BagException &ex) {

    ROS_ERROR("Bag exception : %s", ex.what());
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics), ros::TIME_MIN,
                    ros::TIME_MAX, true);

  ros::Time start = ros::Time::now();
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    if (!ros::ok())
      break;

    if (m.getTopic() == ekf_topic) {
      // ROS_INFO_STREAM("Getting ekf message");
      anm_msgs::VehicleState::ConstPtr vehicle_msg =
          m.instantiate<anm_msgs::VehicleState>();

      if (vehicle_msg != NULL) {
        pose_msg = vehicle_msg.pose.pose;
        // ROS_INFO_STREAM(get_2d_dist(last_pose, pose_msg));
        if (!register_next && (get_2d_dist(last_pose, pose_msg) > dis_inc)) {
          register_next = true;
          last_pose = pose_msg;
        }
      } else {
        ROS_ERROR_STREAM("Could not interpret message from topic "
                         << m.getTopic() << " as nav_msg::Odometry.");
      }
<<<<<<< variant A
      // ROS_INFO_STREAM("Finished with ekf message");
>>>>>>> variant B
======= end
    } else if (m.getTopic() == velo_topic) {
<<<<<<< variant A
      // ROS_INFO_STREAM("Getting velo message");
>>>>>>> variant B
======= end
      if (register_next) {
        counter++;
        if (counter > skip) {
          register_next = false;
          sensor_msgs::PointCloud2::ConstPtr cloud_msg =
              m.instantiate<sensor_msgs::PointCloud2>();
          // call the service for insertion
          laser_slam::InsertPointCloud srv;
          if (cloud_msg != NULL) {
            srv.request.point_cloud = *cloud_msg;
            ROS_INFO_STREAM("Sending pose....");
            ROS_INFO_STREAM(pose_msg.position.x);
            ROS_INFO_STREAM(pose_msg.position.y);
            ROS_INFO_STREAM(pose_msg.position.z);
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
          } else {
            ROS_ERROR("Failed to call service");
            return 1;
          }
        } else {
          ROS_ERROR("Could not get PointCloud2 from message.");
          ros::Duration(3).sleep();
        }
      }
    } else if (m.getTopic() == gps_topic) {
      sensor_msgs::NavSatFix::ConstPtr gps_msg =
          m.instantiate<sensor_msgs::NavSatFix>();
      if (!initialized) {
        initialized = true;
        alt_ref = gps_msg->altitude;
      }
      // pose_msg.position.z = gps_msg->altitude - alt_ref;
    }
  }
  bag.close();

  ROS_INFO_STREAM("All Scans Complete");

  ROS_WARN_STREAM("Execution time: " << (ros::Time::now() - start));
  return 0;
}
