#include <rw/math/Q.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include "Roadmap.hpp"
#include <string>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/math/MetricFactory.hpp>
#include <algorithm>
#include <rw/math/Math.hpp>
#include <time.h> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>


Roadmap::Roadmap(int size, double resolution, double connection_radius, double max_density):

	_resolution(resolution),
	_size(size),
	_connection_radius(connection_radius),
	_max_density(max_density)

{
	if(_workcell == nullptr)
	{
		initWorkCell();
	}

	initRobworkStuff();
	std::stringstream buffer;
	buffer << "Default state: " << _device->getQ(_state) << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
	

};

Roadmap::Roadmap(std::string path)
{
	initWorkCell();
        load_roadmap(path);

	Roadmap(_size, _resolution, _connection_radius, _max_density);

        int non = nonConnectedNodes();

	std::stringstream buffer;
	buffer << "ConnectedNodes: " << _actualSize - non << " , nonConnectedNodes: " << non << std::endl;
	ROS_INFO("%s", buffer.str().c_str());



}

Roadmap::~Roadmap()
{
};

bool Roadmap::inCollision(Node *n)
{
	return _constraint->inCollision(n->q_val);


};


bool Roadmap::inCollision(Node *a, Node *b)
{

	return _edgeConstraint->inCollision(a->q_val, b->q_val);

};

void Roadmap::addNode(rw::math::Q n, int nodeid, bool c, bool u)
{
	Node* tempNode = new Node(n, nodeid);
	tempNode->connected_component = c;
	tempNode->usable = u;
	_graph->push_back(tempNode);
	_kdtree->addNode(n, tempNode);
	 

};

bool Roadmap::addNode()
{
	rw::math::Q sample = _sampler->sample();
	if(sample.empty())
		return false;
	else if(!distanceTooClose(sample))
	{
		addNode(sample, _actualSize, false, false);
		_actualSize++;
		return true;
	}
	return false;

};

void Roadmap::addEdges(int nodeidA, int nodeidB)
{
	std::vector<Node*> t(0);
	t.push_back(_graph->at(nodeidA));
	addEdges(t, _graph->at(nodeidB));
	//_graph->at(nodeidA)->edges.push_back(_graph->at(nodeidB));
	//_graph->at(nodeidB)->edges.push_back(_graph->at(nodeidA));
	//_connectedEdgePairs++;

};

bool Roadmap::distanceTooClose(rw::math::Q a)
{
	_kdnodesSearchResult.clear();


        _kdtree->nnSearchElipse(a, _radi2 ,_kdnodesSearchResult);
	return _kdnodesSearchResult.size();

};

std::vector<Node*> Roadmap::nodesInRange(Node *a)
{
	_kdnodesSearchResult.clear();
	std::vector<Node*> res(0);


	
        _kdtree->nnSearchElipse(a->q_val, _radi ,_kdnodesSearchResult);
	for (std::list<const rwlibs::algorithms::KDTreeQ<Node*>::KDNode*>::const_iterator iterator = _kdnodesSearchResult.begin(), end = _kdnodesSearchResult.end(); iterator != end; ++iterator)
	{
		double dist = _metric->distance(a->q_val, (*iterator)->value->q_val);
		
		if(dist <= _connection_radius)
		{
			std::stringstream buffer;
		buffer << "dist " << dist << std::endl;
		//ROS_ERROR("%s", buffer.str().c_str());
			res.push_back((*iterator)->value);
		}

		



	}
	return res;		

}; 

void Roadmap::addEdges(std::vector<Node*> n, Node *a)
{
	for(int i = 0; i< n.size(); i++)
	{
		bool already_edge = true;
		if(n[i]->edges.size() == 0)
			already_edge = false;
		else
			for(int j = 0; j< n[i]->edges.size(); j++)
			{
				already_edge = false;
				if(n[i]->edges[j]->nodenum == a->nodenum)
				{
					already_edge = true;
					break;
				}
			}
		if(!already_edge && n[i]->nodenum != a->nodenum)
		{
			n[i]->edges.push_back(a);
			a->edges.push_back(n[i]);
			_connectedEdgePairs++;
			//ROS_INFO("Here");

		}	

	}



};

void Roadmap::connectGraph()
{
	for(int i = 0; i< _graph->size(); i++)
	{
		std::vector<Node*> n = nodesInRange(_graph->at(i));
		addEdges(n, _graph->at(i));
	}


};

bool Roadmap::create_roadmap()
{
	while(_actualSize < _size)
	{
		for(int i = 0; i< 100; i++)
			if(addNode())
				break;
			else if(i == 99)
				return false;
		
	}
	
	connectGraph();
	int non = nonConnectedNodes();

	std::stringstream buffer;
	buffer << "ConnectedNodes: " << _actualSize - non << " , nonConnectedNodes: " << non << std::endl;
	ROS_INFO("%s", buffer.str().c_str());


	return true;

};

int Roadmap::nonConnectedNodes()
{
	int count = 0;
	for(int i = 0; i< _graph->size(); i++)
	{
		if(_graph->at(i)->edges.size() == 0)
			count++;
	}

	return count;


};


void Roadmap::initWorkCell()
{

	rw::math::Math::seed();
	std::string path = ros::package::getPath("rovi2") + "/WorkStation_3/WC3_Scene.wc.xml";
	_workcell = rw::loaders::WorkCellLoader::Factory::load(path);

	_device = _workcell->findDevice("UR1");

	_state =  _workcell->getDefaultState();


	// Create Graph
	_graph = new std::vector<Node*>(0);
	_graph->reserve(_size);


	// Create KdTree
	_kdtree = new rwlibs::algorithms::KDTreeQ<Node*>(_device->getBounds().first.size());
	if(_kdtree == NULL)
		ROS_ERROR("KdTree error");


};


void Roadmap::initRobworkStuff()
{
	// Collision checking strategy for collision detection in workcell.
	_strategy = rwlibs::proximitystrategies::ProximityStrategyPQP::make();
	if(_strategy == NULL)
		ROS_ERROR("Strategy error");


	// Detector for collision detection in workcell
	_detector = new rw::proximity::CollisionDetector(_workcell, _strategy);
	if(_detector == NULL)
		ROS_ERROR("Detector error");

	// Constraint, to check for workcell collision.
	_constraint = rw::pathplanning::QConstraint::make(_detector, _device, _state);

	// EdgeConstraint, to check for collision in an edge
	_edgeConstraint = rw::pathplanning::QEdgeConstraint::make(
            _constraint, rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), _resolution);

	// Make a uniform sampler, which only returns collision free samples for both bounds and workcell
	// We might not want a constrained sampler, if we wants to use our own bounds, alternative we can change the robots bounds!
	_sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device),_constraint);
	if(_sampler == NULL)
		ROS_ERROR("Sampler error");

	// Create metric weight, as motion weight in cartesian space.
	_metricWeights = rw::pathplanning::PlannerUtil::estimateMotionWeights(*_device, _device->getEnd(),_state,rw::pathplanning::PlannerUtil::WORSTCASE,1000);

	// Create the metric
	_metric = rw::math::MetricFactory::makeWeightedEuclidean<rw::math::Q>(_metricWeights);

	std::stringstream buffer;
	buffer << "Bounds +/- " << (_device->getBounds().second)[0] << std::endl;
	ROS_INFO("%s", buffer.str().c_str());



	_radi = _metricWeights;

        for(size_t i=0;i<_radi.size();i++){
            _radi[i] = _connection_radius/std::max(_metricWeights(i),0.1);
        }

        double nlen = _radi.norm2();
        for(size_t i=0;i<_radi.size();i++){
            _radi[i] = _radi[i]*(_connection_radius*std::sqrt(_device->getBounds().first.size()))/nlen;
        }


	_radi2 = _metricWeights;

        for(size_t i=0;i<_radi2.size();i++){
            _radi2[i] = _max_density/std::max(_metricWeights(i),0.1);
        }

        nlen = _radi2.norm2();
        for(size_t i=0;i<_radi2.size();i++){
            _radi2[i] = _radi2[i]*(_max_density*std::sqrt(_device->getBounds().first.size()))/nlen;
        }


};

void Roadmap::connectedComponents()
{
	std::vector<Node*> check(0);
	std::vector<std::vector<Node*> > connected_parts(0);
	std::vector<Node*> temp(0);
	for(int i = 0; i< _graph->size(); i++)
	{
		temp.clear();
		if(_graph->at(i)->connected_component == false)
		{
			check.push_back(_graph->at(i));
			//temp.push_back(_graph->at(i));
		}

		while(!check.empty())
		{
			Node* tempNode = check.back();
			check.pop_back();
			if(tempNode->connected_component == false)
			{	
				tempNode->connected_component = true;
				temp.push_back(tempNode);
				for(int j = 0; j < tempNode->edges.size(); j++)
					check.push_back(tempNode->edges.at(j));
			}
			 
		}

		if(temp.size() > 0)
		{
			connected_parts.push_back(temp);
		}

	}

	std::sort(connected_parts.begin(), connected_parts.end(), Sorting);


	for(int i = 0; i< connected_parts.size(); i++)
	{
		std::stringstream buffer;
		buffer << "ConnectedPart " << i+1 << " has " << connected_parts.at(i).size() << " nodes" << std::endl;
		ROS_INFO("%s", buffer.str().c_str());

		if(i == 9)
			break;

	}

	for(int i = 0; i<connected_parts.at(0).size(); i++)
		connected_parts.at(0).at(i)->usable = true;

	_usable_nodes = connected_parts.at(0).size();
	_largestConnected = _usable_nodes;


	// Test
	/*
	for(int i = 0; i<_graph->size(); i++)
		if(_graph->at(i)->usable == true)
		{
			std::stringstream buffer;
			buffer << "Node " << i << " is usable" << std::endl;
			ROS_INFO("%s", buffer.str().c_str());

		}
	*/
	ROS_INFO("Done finding connected parts");
		

};


void Roadmap::save_roadmap(std::string path)
{
	std::ofstream f( ros::package::getPath("rovi2") + "/Roadmap/" + path);
	if ( !f.fail() )
	{
		f << "Size:" << '\t' << std::to_string(_graph->size()) << std::endl;
		f << "Resolution:" << '\t' << std::to_string(_resolution) << std::endl;
		f << "ConnectedEdgePairs:" << '\t' << std::to_string(_connectedEdgePairs) << std::endl;
		f << "Connection_radius:" << '\t' << std::to_string(_connection_radius) << std::endl;
		f << "Max_density:" << '\t' << std::to_string(_max_density) << std::endl;
		f << "Usable_nodes:" << '\t' << std::to_string(_usable_nodes) << std::endl;
		f << "Largest_Connected:" << '\t' << std::to_string(_largestConnected) << std::endl;

		f << "Nodes:" << std::endl;


		for(int i = 0; i< _graph->size(); i++)
		{
			f << std::to_string(i) << '\t' << std::to_string(_graph->at(i)->connected_component) << '\t' << std::to_string(_graph->at(i)->usable) << '\t' << std::to_string(_graph->at(i)->q_val[0]) << '\t' << std::to_string(_graph->at(i)->q_val[1]) << '\t' << std::to_string(_graph->at(i)->q_val[2]) << '\t' << std::to_string(_graph->at(i)->q_val[3]) << '\t' << std::to_string(_graph->at(i)->q_val[4]) << '\t' << std::to_string(_graph->at(i)->q_val[5]) << std::endl;
		}

		f << "Edges:" << std::endl;
		for(int i = 0; i< _graph->size(); i++)
		{
			f << std::to_string(i);
			for(int j = 0; j < _graph->at(i)->edges.size(); j++)
			{
				f << '\t';
				f << std::to_string(_graph->at(i)->edges.at(j)->nodenum);
					
			}
			if(i != (_graph->size()-1))
				f << std::endl;

		}

	}

	f.close();
		


};

void Roadmap::load_roadmap(std::string path)
{
	std::ifstream file(ros::package::getPath("rovi2") + "/Roadmap/" + path);
	std::string   line;
	int switcher = 0;
	while(std::getline(file, line))
	{
		std::vector<std::string> strs;
		boost::split(strs, line, boost::is_any_of("\t "));
		if(switcher == 0)
		{
			if(strs.at(0) == "Size:")
				_size = std::atoi(strs.at(1).c_str());
			else if(strs.at(0) == "Resolution:")
				_resolution = std::atof(strs.at(1).c_str());
			else if(strs.at(0) == "ConnectedEdgePairs:")
			{
				//_connectedEdgePairs = std::atoi(strs.at(1).c_str());
				_connectedEdgePairs = 0;
			}
			else if(strs.at(0) == "Connection_radius:")
				_connection_radius = std::atof(strs.at(1).c_str());
			else if(strs.at(0) == "Max_density:")
				_max_density = std::atof(strs.at(1).c_str());
			else if(strs.at(0) == "Usable_nodes:")
				_usable_nodes = std::atoi(strs.at(1).c_str());
			else if(strs.at(0) == "Largest_Connected:")
				_largestConnected = std::atoi(strs.at(1).c_str());
			else if(strs.at(0) == "Nodes:")
				switcher++;
			else
				ROS_ERROR("Error in roadmap file");
		}
		else if(switcher == 1)
		{
			if(strs.at(0) == "Edges:")
				switcher++;
			else
			{
				int id = std::atoi(strs.at(0).c_str());
				//bool cp = strs.at(1) != "0";
				bool cp = false;
				bool us = strs.at(2) != "0";
				rw::math::Q myQ(6,0,0,0,0,0,0);
				myQ[0] = std::atof(strs.at(3).c_str());
				myQ[1] = std::atof(strs.at(4).c_str());
				myQ[2] = std::atof(strs.at(5).c_str());
				myQ[3] = std::atof(strs.at(6).c_str());
				myQ[4] = std::atof(strs.at(7).c_str());
				myQ[5] = std::atof(strs.at(8).c_str());
				addNode(myQ, id, cp, us);
				_actualSize++;			
			}

		}
		else if(switcher == 2)
		{
			std::vector<Node*> t(0);
			for(int i = 1; i< strs.size(); i++)
				t.push_back(_graph->at(std::atoi(strs.at(i).c_str())));

			addEdges(t, _graph->at(std::atoi(strs.at(0).c_str())));			

		}

	}


}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"roadmap");
  ros::NodeHandle n;
  /*Roadmap Roadmap_ros(20000, 0.005, 1, 0.5);
  if(Roadmap_ros.create_roadmap())
  {
	std::stringstream buffer;
	buffer << "Roadmap created with " << Roadmap_ros._actualSize << " Nodes and " << Roadmap_ros._connectedEdgePairs << " Edge pairs" << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
  }
  else
	ROS_INFO("Could not create Roadmap!");
  
  Roadmap_ros.connectedComponents();
  Roadmap_ros.save_roadmap("test2.txt");
  */





  
   Roadmap Roadmap_ros("test2.txt");
   std::stringstream buffer;
	buffer << "Roadmap created with " << Roadmap_ros._actualSize << " Nodes and " << Roadmap_ros._connectedEdgePairs << " Edge pairs" <<     std::endl;
	ROS_INFO("%s", buffer.str().c_str());
   Roadmap_ros.connectedComponents();





  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
