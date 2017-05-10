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
#ifndef OPTIMIZER_BASE_H
#define OPTIMIZER_BASE_H

#include <laser_slam/graph_slam_types.h>

class GraphSlamOptimizer{
public:
 	//void GraphSlamOptimizer();

 	virtual void initOptimizer()=0; //pure virtual function
 	virtual bool addVertex(Node& node)=0;
 	virtual bool addPrior(Node& node)=0;
 	virtual bool addEdge(Edge& gE)=0;
 	virtual void optimizeGraph()=0;
 	virtual bool removeLastEdges()=0; 
 	
 	void addNode(Node gN)
 	{
 		pose_graph.nodes.push_back(gN);
 		addVertex(gN);
 		
 	}

 	void addPosePrior(Node gN)
 	{
		addPrior(gN);
 	}

 	PoseGraph* getPoseGraph()
 	{
 		return &pose_graph;
 	}

 protected:
 	PoseGraph pose_graph;
 };

#endif
