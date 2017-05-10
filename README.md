# neptec_laser_slam
Laser SLAM Code for the Neptec mapping project

# 1) License

Copyright 2016

* Arun Das (University of Waterloo) [adas@uwaterloo.ca]
* James Servos (University of Waterloo) [jdservos@uwaterloo.ca]
* Steven L Waslander (University of Waterloo) [stevenw@uwaterloo.ca]

All rights reserved.

# 2) Dependencies

* This software requires the ROS gridmap package.  See http://wiki.ros.org/grid_map.  
* It also requires either the g2o or gtsam optimization packages.  Gtsam can be installed from source (https://bitbucket.org/gtborg/gtsam/), while g2o can also be installed from source (https://github.com/RainerKuemmerle/g2o), or easily installed using the Debian package manager system:
```
sudo apt-get install ros-<distro>-libg2o
```

# 3) Service Calls

The laser_slam code runs using the ros service framework (http://wiki.ros.org/Services).  A server node supplies the point cloud and associated point cloud pose in the form of a service call.  The laser_slam package acts as the client node, recieves the point cloud and point cloud pose information, registers these into the global map, and finally returns map data upon completion of the point cloud insertion.  The basic call interface is as follows (can be found in InsertPointCloud.srv):

```
sensor_msgs/PointCloud2 point_cloud
geometry_msgs/Pose point_cloud_pose
---
bool insertion_complete
nav_msgs/OccupancyGrid drivability_map
grid_map_msgs/GridMap elevation_map
sensor_msgs/PointCloud2 full_map_cloud
```
* `point_cloud` is the point cloud to be inserted
* `point_cloud_pose` is the pose `point_cloud` with respect to the slam world frame.  The slam world frame is defined by the first point cloud insertion.
* `insertion_complete` denotes that the map insertion has converged successfully
* `drivability_map` is an occupancy grid map which denotes occupied vs free space for driving
* `evevation_map` is a 2D grid map where each cell contains the elevation in meters, with respect to the the slam world frame
* `full_map_cloud` is an aggregate pointcloud which contains all of the individually registered point clouds

# 4) Basic Usage

To start the laser_slam package, simply use the supplied launch file:

```
roslaunch laser_slam laser_slam.launch
```

A test server is also supplied, which reads point cloud and pose data from data files and issues service calls to the laser_slam package for registration into the global map.  To use, first launch the laser_slam as above, then launch the service tester:

```
roslaunch laser_slam laser_slam_service_tester.launch
```

# 4) Visualization

The output topics can be visualized in rivz.  An rviz configuration file is provided in the `laser_slam/config` folder

# 5) Paramter tuning

Tuning parameters for the laser_slam package are set in `laser_slam/config/laser_slam_config.yaml` file.  Descriptions of the parameters are provided within the file.  
