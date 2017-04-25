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


Roadmap::Roadmap(int size, double resolution, double connection_radius, double max_density):

	_resolution(resolution),
	_size(size),
	_connection_radius(connection_radius),
	_max_density(max_density)

{
	Roadmap::initWorkCell();
	std::stringstream buffer;
	buffer << "Default state: " << _device->getQ(_state) << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
	

};

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

void Roadmap::addNode(rw::math::Q n, int nodeid)
{
	Node* tempNode = new Node(n, nodeid);
	_graph->push_back(tempNode);
	_kdtree->addNode(n, tempNode);
	 

};

bool Roadmap::addNode()
{
	rw::math::Q sample = _sampler->sample();
	if(!distanceTooClose(sample))
	{
		addNode(sample, _actualSize);
		_actualSize++;
		return true;
	}
	return false;

};

void Roadmap::addEdges(int nodeidA, int nodeidB)
{
	_graph->at(nodeidA)->edges.push_back(_graph->at(nodeidB));
	_graph->at(nodeidB)->edges.push_back(_graph->at(nodeidA));
	_connectedEdgePairs++;

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

	// Create KdTree
	_kdtree = new rwlibs::algorithms::KDTreeQ<Node*>(_device->getBounds().first.size());
	if(_kdtree == NULL)
		ROS_ERROR("KdTree error");

	// Create Graph
	_graph = new std::vector<Node*>(0);
	_graph->reserve(_size);


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
	std::vector<int> connected_parts(0);
	int connectedNum = 0;
	for(int i = 0; i< _graph->size(); i++)
	{
		if(_graph->at(i)->connected_component == false)
			check.push_back(_graph->at(i));

		while(!check.empty())
		{
			Node* tempNode = check.back();
			check.pop_back();
			if(tempNode->connected_component == false)
			{	
				tempNode->connected_component = true;
				connectedNum++;
				for(int j = 0; j < tempNode->edges.size(); j++)
					check.push_back(tempNode->edges.at(j));
			}
			 
		}

		if(connectedNum > 0)
		{
			connected_parts.push_back(connectedNum);
			connectedNum = 0;
		}

	}

	std::sort(connected_parts.begin(), connected_parts.end(), Sorting);


	for(int i = 0; i< connected_parts.size(); i++)
	{
		std::stringstream buffer;
		buffer << "ConnectedPart " << i+1 << " has " << connected_parts.at(i) << " nodes" << std::endl;
		ROS_INFO("%s", buffer.str().c_str());

		if(i == 9)
			break;

	}

	ROS_INFO("Done finding connected parts");
		

};


int main(int argc, char **argv)
{
  ros::init(argc,argv,"roadmap");
  ros::NodeHandle n;
  Roadmap Roadmap_ros(100, 0.005, 1, 0.5);
  if(Roadmap_ros.create_roadmap())
  {
	std::stringstream buffer;
	buffer << "Roadmap created with " << Roadmap_ros._actualSize << " Nodes and " << Roadmap_ros._connectedEdgePairs << " Edge pairs" << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
  }
  else
	ROS_INFO("Could not create Roadmap!");

  Roadmap_ros.connectedComponents();

  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
