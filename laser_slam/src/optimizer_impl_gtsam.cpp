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
#include <laser_slam/optimizer_impl_gtsam.h>

using namespace std;
using namespace gtsam;

 GraphSlamOptimizerGTSAM::GraphSlamOptimizerGTSAM()
 {
    graph.resize(0);
    initial.clear();
    initOptimizer();
    /*linearSolver = new SlamLinearSolver();
    blockSolver = new SlamBlockSolver(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    initOptimizer();*/
 }

void GraphSlamOptimizerGTSAM::initOptimizer()
{
    graph.resize(0);
    initial.clear();
    pose_graph.nodes.clear();
    pose_graph.edges.clear();
}

bool GraphSlamOptimizerGTSAM::addVertex(Node& node)
{

    Key id = node.idx;
    Rot3 R = Rot3::quaternion(node.pose.pose.orientation.w, node.pose.pose.orientation.x, node.pose.pose.orientation.y, node.pose.pose.orientation.z);
    Point3 t = Point3(node.pose.pose.position.x, node.pose.pose.position.y, node.pose.pose.position.z);
    initial.insert(id, Pose3(R,t));

    if(node.idx ==0) //first one, add a prior factor
    {
      //ROS_INFO_STREAM("adding prior factor...");
      Vector6 v6;
      v6<< 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
      noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances(v6);
      Key firstKey = 0;
      graph.add(PriorFactor<Pose3>(firstKey, Pose3(R,t), priorModel));
    }
    return true;

    /*const double pose[7] = {node.pose.pose.position.x, node.pose.pose.position.y, node.pose.pose.position.z, node.pose.pose.orientation.x, node.pose.pose.orientation.y, node.pose.pose.orientation.z, node.pose.pose.orientation.w};
    g2o::VertexSE3* robot =  new g2o::VertexSE3;
    robot->setId(node.idx);
    robot->setEstimateDataImpl(pose);
    return optimizer.addVertex(robot);*/
}

bool GraphSlamOptimizerGTSAM::addEdge(Edge& gE)
{

    
    Key id1 = gE.from; 
    Key id2 = gE.to;
    Rot3 R = Rot3::quaternion(gE.edge.pose.orientation.w, gE.edge.pose.orientation.x, gE.edge.pose.orientation.y, gE.edge.pose.orientation.z);
    Point3 t = Point3(gE.edge.pose.position.x, gE.edge.pose.position.y, gE.edge.pose.position.z);
    
    Matrix m = gE.edgeInf;
    Matrix mgtsam = eye(6);
    mgtsam.block(0,0,3,3) = m.block(3,3,3,3); // cov rotation
    mgtsam.block(3,3,3,3) = m.block(0,0,3,3); // cov translation
    mgtsam.block(0,3,3,3) = m.block(0,3,3,3); // off diagonal
    mgtsam.block(3,0,3,3) = m.block(3,0,3,3); // off diagonal
    SharedNoiseModel model = noiseModel::Gaussian::Information(mgtsam);
    NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id1, id2, Pose3(R,t), model));
    graph.push_back(factor);
    return true;

    /*const double transf[7] = {gE.edge.pose.position.x,gE.edge.pose.position.y, gE.edge.pose.position.z, gE.edge.pose.orientation.x,gE.edge.pose.orientation.y,gE.edge.pose.orientation.z, gE.edge.pose.orientation.w};

    g2o::EdgeSE3* edge = new g2o::EdgeSE3;
    edge->vertices()[0] = optimizer.vertex(gE.from);
    edge->vertices()[1] = optimizer.vertex(gE.to);

    edge->setMeasurementData(transf);
    Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6,6);
    info = gE.edgeInf;;
    edge->setInformation(info);
    return optimizer.addEdge(edge);*/
}

void GraphSlamOptimizerGTSAM::optimizeGraph(){

  ROS_INFO_STREAM("Optimizer::Optimizing the factor graph");
  LevenbergMarquardtParams params;
  params.setVerbosity("TERMINATION"); // this will show info about stopping conditions
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();
  ROS_INFO_STREAM("Optimizer::Optimization complete");
  
  ROS_INFO_STREAM("GTSAM::Initial error= " << graph.error(initial));
  ROS_INFO_STREAM("GTSAM::Final error= " << graph.error(result));

  //fill in the pose graph

  for(unsigned int i=0; i<pose_graph.nodes.size(); i++){

    //ROS_INFO_STREAM("key: " << i);
    //gtsam::Symbol s('x', (i+1));
    gtsam::Pose3 pose3D = result.at<gtsam::Pose3>(i);

    //std::cout<< pose3D <<std::endl;
    //ROS_INFO_STREAM("here1");
    Point3 p = pose3D.translation();
    Rot3 R = pose3D.rotation();
    //ROS_INFO_STREAM("here2");

    pose_graph.nodes[i].pose.pose.position.x = p.x();
    pose_graph.nodes[i].pose.pose.position.y = p.y();
    pose_graph.nodes[i].pose.pose.position.z = p.z();

    pose_graph.nodes[i].pose.pose.orientation.x = R.toQuaternion().x();
    pose_graph.nodes[i].pose.pose.orientation.y = R.toQuaternion().y();
    pose_graph.nodes[i].pose.pose.orientation.z = R.toQuaternion().z();
    pose_graph.nodes[i].pose.pose.orientation.w = R.toQuaternion().w();
    
  }

  

    //set first pose fixed
    /*g2o::VertexSE3* firstRobotPose = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);

    ROS_INFO_STREAM("Optimizing");
    optimizer.initializeOptimization();

    optimizer.computeActiveErrors();
    ROS_INFO_STREAM("Initial chi2 = " << FIXED(optimizer.chi2()));
    optimizer.optimize(100);
    
    for (size_t i = 0; i < pose_graph.nodes.size(); ++i) {
        g2o::VertexSE3* v = vertex(i);
        double est[7];
        if(v == NULL) {
            continue;
        }

        v->getEstimateData(est);
               
         pose_graph.nodes[i].pose.pose.position.x = est[0];
         pose_graph.nodes[i].pose.pose.position.y = est[1];
         pose_graph.nodes[i].pose.pose.position.z = est[2];
         pose_graph.nodes[i].pose.pose.orientation.x =est[3]; 
         pose_graph.nodes[i].pose.pose.orientation.y =est[4]; 
         pose_graph.nodes[i].pose.pose.orientation.z =est[5]; 
         pose_graph.nodes[i].pose.pose.orientation.w = est[6]; 
    }
            
    ROS_INFO_STREAM("optimization done.");
    optimizer.computeActiveErrors();
    ROS_INFO_STREAM("post chi2 = " << FIXED(optimizer.chi2()));*/

}
