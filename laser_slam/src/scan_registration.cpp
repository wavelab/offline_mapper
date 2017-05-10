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
#include <laser_slam/scan_registration.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

struct sortContainer
{
	double distance;
	int idx;

};

bool distSort (sortContainer p1,sortContainer p2) 
{ 
    return (p1.distance < p2.distance); 
}

void ScanRegistration::setup() {
    
    // Set the maximum number of iterations (criterion 1)
    registration_icp.setMaximumIterations (200);
    // Set the transformation epsilon (criterion 2)
    registration_icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    registration_icp.setEuclideanFitnessEpsilon (1e-8);
    registration_icp.setRANSACIterations(100);
}



double ScanRegistration::getNearestKF(PoseGraph& pG, Node& gN)
{
	int numKF = pG.nodes.size();
	double smallestDist = 1e20;
	double dx = 0;
	double dy=0;
	
	for(int i=0; i<numKF; i++)
	{
		Node currNode = pG.nodes[i];
		dx = currNode.pose.pose.position.x - gN.pose.pose.position.x; dy = currNode.pose.pose.position.y - gN.pose.pose.position.y;
		double currDist = sqrt( dx*dx + dy*dy);
		
		if(currDist<smallestDist && gN.idx!=currNode.idx) 
		{
			smallestDist = currDist;
			
		}
	}
	
	return smallestDist;
}

std::vector<int> ScanRegistration::getKNN(PoseGraph& pG, Node& gN, unsigned int K)
{

	std::vector<int> toReturn;
	std::vector<sortContainer> sCVector;
	int numKF = pG.nodes.size();
	
	double dx = 0;
	double dy=0;
    
    K = std::min(numKF,(int)K); 

	//grab the KNN
	for(int i=0; i<numKF; i++)
	{
		sortContainer sC;
		Node currNode = pG.nodes[i];
		dx = currNode.pose.pose.position.x - gN.pose.pose.position.x; dy = currNode.pose.pose.position.y - gN.pose.pose.position.y;
		double currDist = sqrt( dx*dx + dy*dy);
		sC.distance = currDist;
		sC.idx = i;
		sCVector.push_back(sC);
	}
	
	sort(sCVector.begin(),sCVector.end(),distSort);

	for(unsigned int i=0; i<(K);i++)
	{
		if(i < sCVector.size() && gN.idx!=sCVector[i].idx)	{	
			toReturn.push_back(sCVector[i].idx);
	    }
	}
	
	return toReturn;
}

Eigen::Matrix6d ScanRegistration::computeEdgeInformationLUM(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_trans, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, double max_corr_distance)
{
	//this function assumes source_could is transformed by the pose provided by the scan reg algo
	
   int numSourcePts = source_trans->size();
   std::vector <Eigen::Vector3f> corrs_aver (numSourcePts);
   std::vector <Eigen::Vector3f> corrs_diff (numSourcePts);
   int numCorr = 0;
   
   Eigen::Matrix6d edgeCov = Eigen::Matrix6d::Identity ();
   
   //build kd tree for source points
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(target);
  
  //iterate through the source cloud and compute match covariance
  
  for(int i=0; i<numSourcePts; i++)
  {
	  pcl::PointXYZ qpt = source_trans->points[i];
	  std::vector<int> nn_idx;
	  std::vector<float> nn_sqr_dist;
	  kdtree.nearestKSearch(qpt,1,nn_idx,nn_sqr_dist); //returns the index of the nn point in the target
	  
	if(nn_sqr_dist[0]<max_corr_distance*max_corr_distance) //if the distance to point is less than max correspondence distance, use it to calculate  
	{
		Eigen::Vector3f source_pt = qpt.getVector3fMap();
		Eigen::Vector3f target_pt = target->points[nn_idx[0]].getVector3fMap();
		
		// Compute the point pair average and difference and store for later use
		corrs_aver[numCorr] = 0.5 * (source_pt + target_pt);
		corrs_diff[numCorr] = source_pt - target_pt;
		numCorr++;
	}
	else
	{
		continue;
	}
   }
	corrs_aver.resize (numCorr);
	corrs_diff.resize (numCorr);
	
	//now compute the M matrix
	Eigen::Matrix6d MM = Eigen::Matrix6d::Zero ();
	Eigen::Vector6d MZ = Eigen::Vector6d::Zero ();
	for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
	{
		// Fast computation of summation elements of M'M
		MM (0, 4) -= corrs_aver[ci] (1);
		MM (0, 5) += corrs_aver[ci] (2);
		MM (1, 3) -= corrs_aver[ci] (2);
		MM (1, 4) += corrs_aver[ci] (0);
		MM (2, 3) += corrs_aver[ci] (1);
		MM (2, 5) -= corrs_aver[ci] (0);
		MM (3, 4) -= corrs_aver[ci] (0) * corrs_aver[ci] (2);
		MM (3, 5) -= corrs_aver[ci] (0) * corrs_aver[ci] (1);
		MM (4, 5) -= corrs_aver[ci] (1) * corrs_aver[ci] (2);
		MM (3, 3) += corrs_aver[ci] (1) * corrs_aver[ci] (1) + corrs_aver[ci] (2) * corrs_aver[ci] (2);
		MM (4, 4) += corrs_aver[ci] (0) * corrs_aver[ci] (0) + corrs_aver[ci] (1) * corrs_aver[ci] (1);
		MM (5, 5) += corrs_aver[ci] (0) * corrs_aver[ci] (0) + corrs_aver[ci] (2) * corrs_aver[ci] (2);
		
		// Fast computation of M'Z
		MZ (0) += corrs_diff[ci] (0);
		MZ (1) += corrs_diff[ci] (1);
		MZ (2) += corrs_diff[ci] (2);
		MZ (3) += corrs_aver[ci] (1) * corrs_diff[ci] (2) - corrs_aver[ci] (2) * corrs_diff[ci] (1);
		MZ (4) += corrs_aver[ci] (0) * corrs_diff[ci] (1) - corrs_aver[ci] (1) * corrs_diff[ci] (0);
		MZ (5) += corrs_aver[ci] (2) * corrs_diff[ci] (0) - corrs_aver[ci] (0) * corrs_diff[ci] (2);
    
	}
	// Remaining elements of M'M
	MM (0, 0) = MM (1, 1) = MM (2, 2) = static_cast<float> (numCorr);
	MM (4, 0) = MM (0, 4);
	MM (5, 0) = MM (0, 5);
	MM (3, 1) = MM (1, 3);
	MM (4, 1) = MM (1, 4);
	MM (3, 2) = MM (2, 3);
	MM (5, 2) = MM (2, 5);
	MM (4, 3) = MM (3, 4);
	MM (5, 3) = MM (3, 5);
	MM (5, 4) = MM (4, 5);
	
	// Compute pose difference estimation
	Eigen::Vector6d D = static_cast<Eigen::Vector6d> (MM.inverse () * MZ);

	// Compute s^2
	float ss = 0.0f;
	for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
	{
		ss += static_cast<float> (pow (corrs_diff[ci] (0) - (D (0) + corrs_aver[ci] (2) * D (5) - corrs_aver[ci] (1) * D (4)), 2.0f)
                            + pow (corrs_diff[ci] (1) - (D (1) + corrs_aver[ci] (0) * D (4) - corrs_aver[ci] (2) * D (3)), 2.0f)
                            + pow (corrs_diff[ci] (2) - (D (2) + corrs_aver[ci] (1) * D (3) - corrs_aver[ci] (0) * D (5)), 2.0f));
     }

  // When reaching the limitations of computation due to linearization
  if (ss < 0.0000000000001 || !pcl_isfinite (ss))
  {
    ROS_INFO_STREAM( "Warning: Edge Covariance is singular or illdefined" );
    edgeCov = Eigen::Matrix6d::Identity();
    return edgeCov;
  }

  // Store the results in the slam graph
	edgeCov = MM * (1.0f / ss);
  	
  	return edgeCov; 
 }
  


bool ScanRegistration::calcEdgeIcp(Edge& gE, PoseGraph& pG, double scale_div) 
{
    geometry_msgs::PoseStamped edge;
    geometry_msgs::PoseStamped initPose;
    
    Node& nodeTo = pG.nodes[gE.to];
    Node& nodeFrom = pG.nodes[gE.from];
    
    Eigen::Affine3d Tto,Tfrom;
    tf::poseMsgToEigen (nodeTo.pose.pose, Tto);
    tf::poseMsgToEigen (nodeFrom.pose.pose, Tfrom);
    
    Eigen::Matrix4d Mto = Tto.matrix(); 
    Eigen::Matrix4d Mfrom = Tfrom.matrix(); 

    // Set the max correspondence distance 
    registration_icp.setMaxCorrespondenceDistance (course_scale_icp_distance_max/scale_div);
        
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation = (Mfrom.inverse()*Mto).cast<float>();
    Eigen::Affine3d initialization_pose(transformation.cast<double>());
    tf::poseEigenToMsg (initialization_pose, initPose.pose);
    pcl::PointCloud<pcl::PointXYZ>::Ptr from_cld(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr to_cld(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize (double(course_scale_initial_leaf_size/(scale_div)), double(course_scale_initial_leaf_size/(scale_div)), double(course_scale_initial_leaf_size/scale_div));
    registration_icp.setMaxCorrespondenceDistance (course_scale_icp_distance_max/(scale_div));
    sor.setInputCloud(nodeFrom.keyframe);
    sor.filter (*from_cld); 
    sor.setInputCloud(nodeTo.keyframe);
    sor.filter (*to_cld); 
    ROS_INFO_STREAM("Scan Registraion::Reference scan size: " << to_cld->size());
    ROS_INFO_STREAM("Scan Registraion::Target scan size: "<< from_cld->size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    registration_icp.setInputSource(to_cld);
    registration_icp.setInputTarget(from_cld);
    registration_icp.align(*temp, transformation);
    transformation =  registration_icp.getFinalTransformation();
    
    pcl::transformPointCloud(*to_cld,*temp, transformation); //transform cloud
    Eigen::Matrix6d edge_information;
    edge_information = computeEdgeInformationLUM(temp,from_cld,course_scale_icp_distance_max/scale_div); //the max here should be the actual icp dmax
   
      
    edge.pose.position.x = static_cast<double>(transformation(0,3));
    edge.pose.position.y = static_cast<double>(transformation(1,3));
    edge.pose.position.z = static_cast<double>(transformation(2,3));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)), 
        static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)), 
        static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

    tf::Quaternion qfin;
    tf3d.getRotation(qfin);
    tf::quaternionTFToMsg(qfin,edge.pose.orientation);     
        
    double x_diff = fabs(initPose.pose.position.x -  edge.pose.position.x);
    double y_diff = fabs(initPose.pose.position.y -  edge.pose.position.y);
    double theta_diff = fabs(tf::getYaw(initPose.pose.orientation) -  tf::getYaw(edge.pose.orientation));
    
    if(theta_diff > 2*M_PI)
        theta_diff = theta_diff-2*M_PI;
    else if(theta_diff > M_PI)
        theta_diff = 2*M_PI-theta_diff;

    gE.edge = edge;
    gE.edgeInf  = edge_information; 

   //check if it's a bad match based on dist thresh
   if( (x_diff >  icp_trans_threshold) || (y_diff > icp_trans_threshold) || (theta_diff> icp_rot_threshold) )//reject
   {
	ROS_WARN_STREAM("Scan Registration::Failed ICP match!");
	ROS_INFO_STREAM("x_diff:"<<x_diff);
	ROS_INFO_STREAM("y_diff:"<<y_diff);
	ROS_INFO_STREAM("t_diff:"<<theta_diff);
	return false;
   }
   else
	return true;
}


bool ScanRegistration::addEdgeToX(PoseGraph& pG, int from, int to,double scale_div)
{
	bool isGoodMatch = false;
	if(pG.nodes.size()>=2)
	{

		Edge gE;
		gE.from = from;
		gE.to = to;
	    gE.ctype =2;
	
	    //do icp
	    isGoodMatch= calcEdgeIcp(gE,pG,scale_div);
	   if(isGoodMatch)
	   {
           pG.edges.push_back(gE);
	   }
	}
    return isGoodMatch;
}
