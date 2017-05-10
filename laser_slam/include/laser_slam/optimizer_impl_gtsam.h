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
#ifndef OPTIMIZER_IMPL_GTSAM_H
#define OPTIMIZER_IMPL_GTSAM_H

#include <laser_slam/optimizer_base.h>
#include <laser_slam/graph_slam_types.h>

 //gtsam headers
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>


using namespace std;
using namespace gtsam;

 class GraphSlamOptimizerGTSAM : public GraphSlamOptimizer 
 {
 public:
    GraphSlamOptimizerGTSAM();

    void initOptimizer();
    bool addVertex(Node& node);
    bool addEdge(Edge& gE);
    bool addPrior(Node& node)
    {
    	// Not yet implemented for GTSAM
    	ROS_ERROR("Not yet implemented!");
    }
    bool removeLastEdges(){
    	// Not yet implemented for GTSAM
    	ROS_ERROR("Not yet implemented!");
    }
    void optimizeGraph();
  private:

  NonlinearFactorGraph graph;
  Values initial;

	
 };

#endif
