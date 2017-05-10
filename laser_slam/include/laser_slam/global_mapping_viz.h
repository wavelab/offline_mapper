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

#include <ros/ros.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <Eigen/Core>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <laser_slam/graph_slam_types.h>


void graphVizInit();
visualization_msgs::MarkerArray getVisualMsg(NodeList& pG, EdgeList& edgeList); 

#endif
