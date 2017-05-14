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
#include <laser_slam/optimizer_impl_g2o.h>

GraphSlamOptimizerG2O::GraphSlamOptimizerG2O() {
  linearSolver = new SlamLinearSolver();
  blockSolver = new SlamBlockSolver(linearSolver);
  solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  initOptimizer();
}

void GraphSlamOptimizerG2O::initOptimizer() {
  optimizer.clear();
  linearSolver->setBlockOrdering(false);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);
  pose_graph.nodes.clear();
  pose_graph.edges.clear();

  // Setup the parameter offsets. This step is necessary
  // as described here: https://github.com/RainerKuemmerle/g2o/issues/34
  // Else, we get a std::bad_typeid exception at runtime.
  g2o::ParameterSE3Offset *pOffset = new g2o::ParameterSE3Offset;
  pOffset->setId(0);
  optimizer.addParameter(pOffset);
}

bool GraphSlamOptimizerG2O::addVertex(Node &node) {
  const double pose[7] = {
      node.pose.pose.position.x,    node.pose.pose.position.y,
      node.pose.pose.position.z,    node.pose.pose.orientation.x,
      node.pose.pose.orientation.y, node.pose.pose.orientation.z,
      node.pose.pose.orientation.w};
  g2o::VertexSE3 *robot = new g2o::VertexSE3;
  robot->setId(node.idx);
  robot->setEstimateDataImpl(pose);

  return optimizer.addVertex(robot);
}

bool GraphSlamOptimizerG2O::removeLastEdges() {
  int num_vert = pose_graph.nodes.size() - 1;
  g2o::VertexSE3 *v = vertex(num_vert);
  std::set<g2o::HyperGraph::Edge *> edge_set = v->edges();

  for (auto edge : edge_set) {
    optimizer.removeEdge(edge);
  }

  return true;
}

bool GraphSlamOptimizerG2O::addPrior(Node &node) {
  ROS_INFO_STREAM("Adding prior.");
  g2o::EdgeSE3Prior *prior_edge = new g2o::EdgeSE3Prior;
  // Get the vertex for which we want to add the prior.
  prior_edge->vertices()[0] = optimizer.vertex(node.idx);
  // Add the prior on the vertex.

  double prior_data[7] = {0};
  prior_data[0] = static_cast<double>(node.prior_pose(0, 3));
  prior_data[1] = static_cast<double>(node.prior_pose(1, 3));
  prior_data[2] = static_cast<double>(node.prior_pose(2, 3));
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(node.prior_pose(0, 0)),
                static_cast<double>(node.prior_pose(0, 1)),
                static_cast<double>(node.prior_pose(0, 2)),
                static_cast<double>(node.prior_pose(1, 0)),
                static_cast<double>(node.prior_pose(1, 1)),
                static_cast<double>(node.prior_pose(1, 2)),
                static_cast<double>(node.prior_pose(2, 0)),
                static_cast<double>(node.prior_pose(2, 1)),
                static_cast<double>(node.prior_pose(2, 2)));
  tf::Quaternion qfin;
  tf3d.getRotation(qfin);
  prior_data[3] = static_cast<double>(qfin.x());
  prior_data[4] = static_cast<double>(qfin.y());
  prior_data[5] = static_cast<double>(qfin.z());
  prior_data[6] = static_cast<double>(qfin.w());

  prior_edge->setMeasurementData(prior_data);
  // \todo(adas) remove these magic numbers and use proper
  // information values.
  Eigen::MatrixXd info = (1 / 0.005) * Eigen::MatrixXd::Identity(6, 6);
  info(3, 3) = 100;
  info(4, 4) = 100;
  info(5, 5) = 0.1;
  prior_edge->setInformation(info);
  prior_edge->setParameterId(0, 0);
  return optimizer.addEdge(prior_edge);
}

bool GraphSlamOptimizerG2O::addEdge(Edge &gE) {
  const double transf[7] = {
      gE.edge.pose.position.x,    gE.edge.pose.position.y,
      gE.edge.pose.position.z,    gE.edge.pose.orientation.x,
      gE.edge.pose.orientation.y, gE.edge.pose.orientation.z,
      gE.edge.pose.orientation.w};

  g2o::EdgeSE3 *edge = new g2o::EdgeSE3;
  edge->vertices()[0] = optimizer.vertex(gE.from);
  edge->vertices()[1] = optimizer.vertex(gE.to);

  edge->setMeasurementData(transf);
  Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6);
  info = gE.edgeInf;
  edge->setInformation(info);
  return optimizer.addEdge(edge);
}

void GraphSlamOptimizerG2O::optimizeGraph() {

  // set first pose fixed
  g2o::VertexSE3 *firstRobotPose =
      dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(0));
  // firstRobotPose->setFixed(true);

  ROS_INFO_STREAM("Optimizing");
  optimizer.initializeOptimization();

  optimizer.computeActiveErrors();
  ROS_INFO_STREAM("Initial chi2 = " << FIXED(optimizer.chi2()));
  optimizer.optimize(100);

  Node preNode = pose_graph.nodes[pose_graph.nodes.size() - 1];
  for (size_t i = 0; i < pose_graph.nodes.size(); ++i) {
    g2o::VertexSE3 *v = vertex(i);
    double est[7];
    if (v == NULL) {
      continue;
    }

    v->getEstimateData(est);

    pose_graph.nodes[i].pose.pose.position.x = est[0];
    pose_graph.nodes[i].pose.pose.position.y = est[1];
    pose_graph.nodes[i].pose.pose.position.z = est[2];
    pose_graph.nodes[i].pose.pose.orientation.x = est[3];
    pose_graph.nodes[i].pose.pose.orientation.y = est[4];
    pose_graph.nodes[i].pose.pose.orientation.z = est[5];
    pose_graph.nodes[i].pose.pose.orientation.w = est[6];
  }

  ROS_INFO_STREAM("optimization done.");
  optimizer.computeActiveErrors();
  ROS_INFO_STREAM("post chi2 = " << FIXED(optimizer.chi2()));
}
