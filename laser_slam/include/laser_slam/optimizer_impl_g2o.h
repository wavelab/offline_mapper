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
#ifndef OPTIMIZER_IMPL_G2O_H
#define OPTIMIZER_IMPL_G2O_H

// g2o headers
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include <laser_slam/graph_slam_types.h>
#include <laser_slam/optimizer_base.h>
#include <tf/tf.h>

#include <ros/ros.h>

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>
    SlamLinearSolver;

class GraphSlamOptimizerG2O : public GraphSlamOptimizer {
public:
  GraphSlamOptimizerG2O();

  void initOptimizer();
  bool addVertex(Node &node);
  bool addEdge(Edge &gE);
  bool addPrior(Node &node);
  bool removeLastEdges();
  void optimizeGraph();

private:
  g2o::VertexSE3 *vertex(int i) {
    return dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
  }

  g2o::SparseOptimizer optimizer;
  SlamLinearSolver *linearSolver;
  SlamBlockSolver *blockSolver;
  g2o::OptimizationAlgorithmLevenberg *solver;
};

#endif
