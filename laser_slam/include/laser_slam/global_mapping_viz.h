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
#ifndef GLOBAL_MAPPING_VIZ_H
#define GLOBAL_MAPPING_VIZ_H

#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <geometry_msgs/PolygonStamped.h>
#include <laser_slam/graph_slam_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

void graphVizInit();
visualization_msgs::MarkerArray getVisualMsg(NodeList &pG, EdgeList &edgeList);

#endif
