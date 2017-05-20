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
#include <boost/chrono.hpp>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <iomanip>
#include "Astar.hpp"



Roadmap::Roadmap(ros::NodeHandle h, int size, double resolution, double connection_radius, double max_density):

	_resolution(resolution),
	_size(size),
	_connection_radius(connection_radius),
	_max_density(max_density),
	_nodehandle(h)

{

	initWorkCell();
	
	initRobworkStuff();

	threads.push_back(nullptr);
	threads.push_back(nullptr);
	threads.push_back(nullptr);
	threads.push_back(nullptr);

	std::stringstream buffer;
	buffer << "Default state: " << _device1->getQ(_state1) << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
	

};

Roadmap::Roadmap(ros::NodeHandle h, std::string path)
{
	initWorkCell();

        load_roadmap(path);
	initRobworkStuff();

	std::stringstream buffer1;
	buffer1 << "RobotEnd " << *(_device1->getEnd()) << std::endl;
	ROS_INFO("%s", buffer1.str().c_str());
	

	//Roadmap(h, _size, _resolution, _connection_radius, _max_density);

        int non = nonConnectedNodes();

	std::stringstream buffer;
	buffer << "ConnectedNodes: " << _actualSize - non << " , nonConnectedNodes: " << non << std::endl;
	ROS_INFO("%s", buffer.str().c_str());

	planner = new Astar(_size, _graph, _metric, _constraintAstar, _edgeConstraintAstar);
	service_start_plan = _nodehandle.advertiseService("rovi2/Roadmap/StartPlan", &Roadmap::start_plan, this);
	path_publisher = _nodehandle.advertise<rovi2::path>("rovi2/Roadmap/Path", 1);
	service_next_conf = _nodehandle.advertiseService("rovi2/Roadmap/NextConf", &Roadmap::check_plan, this);
	


}

Roadmap::~Roadmap()
{
	for(int i = 0; i< _graph->size(); i++)
		delete _graph->at(i);
	delete _graph;

	delete _workcell1;
  	delete _workcell2;
  	delete _workcell3;
  	delete _workcell4;
	delete _workcellAstar;
  	delete _device1;
  	delete _device2;
  	delete _device3;
  	delete _device4;
	delete _deviceAstar;
	delete _detector1;
	delete _detector2;
	delete _detector3;
	delete _detector4;
	delete _detectorAstar;

	/*delete _constraint1;
	delete _constraint2;
	delete _constraint3;
	delete _constraint4;
	delete _constraintAstar;
	delete _edgeConstraint1;
	delete _edgeConstraint2;
	delete _edgeConstraint3;
	delete _edgeConstraint4;
	delete _edgeConstraintAstar;
	delete _strategy1;
	delete _strategy2;
	delete _strategy3;
	delete _strategy4;
	delete _strategyAstar;*/
	delete _metric;
	delete _kdtree;
	//delete _sampler;
	/*if(planner != nullptr)
		delete planner;
	if(astar_thread != nullptr)
		delete astar_thread;*/
};

/************************************************************************
 * Q  -> lend from Caros
 ************************************************************************/
rw::math::Q Roadmap::toRw(const rovi2::Q& q)
{
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

rovi2::Q Roadmap::toRos(const rw::math::Q& q)
{
  rovi2::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}

bool Roadmap::check_plan(rovi2::Conf::Request & request, rovi2::Conf::Response &res)
{
	if(!_constraintAstar->inCollision(_graph->at(request.second)->q_val))
	{
		if(!_edgeConstraintAstar->inCollision(_graph->at(request.first)->q_val, _graph->at(request.second)->q_val))
		{
			res.target = toRos(_graph->at(request.first)->q_val);
			res.success = true;
			return true;

		}

	}

	res.success = false;
	return true;


}


bool Roadmap::start_plan(rovi2::Plan::Request & request, rovi2::Plan::Response &res)
{
	if(astar_thread != nullptr)
	{
		if(astar_thread->try_join_for(boost::chrono::milliseconds(1)))
		{
			delete astar_thread;
			astar_thread = nullptr;
		}
		else
			res.success = false;

	}


	if(astar_thread == nullptr)
	{
		rw::math::Q init = Roadmap::toRw(request.init);
		rw::math::Q goal = Roadmap::toRw(request.goal);
		std::vector<int> path(0);
		astar_thread = new boost::thread(boost::bind(&Roadmap::find_path, this, init, goal));
		res.success = true;
	}



	
	return true;

}

void Roadmap::find_path(rw::math::Q init, rw::math::Q goal)
{
	int initId = nodesInRange(init);
	int goalId = nodesInRange(goal);
	rovi2::path path;
	if(initId != NULL && goalId != NULL && initId != goalId)
		planner->find_path(initId, goalId, path);


	
	path_publisher.publish(path);
	std::stringstream path_length;
	//path_length << "Path_length: " << path.data.size() << std::endl;
	//ROS_INFO("%s", path_length.str().c_str());
	
	


}

bool Roadmap::inCollision(Node *n)
{
	return _constraint1->inCollision(n->q_val);


};


bool Roadmap::inCollision(Node *a, Node *b)
{

	return _edgeConstraint1->inCollision(a->q_val, b->q_val);

};

void Roadmap::addNode(rw::math::Q n, int nodeid, bool c, bool u)
{
	Node* tempNode = new Node(n, nodeid);
	tempNode->connected_component = c;
	tempNode->usable = u;
	_graph->push_back(tempNode);
	if(u)
		_kdtree->addNode(n, tempNode);
	 

};

void Roadmap::addNode(rw::math::Q n, int nodeid)
{
	Node* tempNode = new Node(n, nodeid);
	tempNode->connected_component = false;
	tempNode->usable = false;
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
		addNode(sample, _actualSize);
		_actualSize++;
		return true;
	}
	return false;

};

void Roadmap::addEdges(int nodeidA, int nodeidB)
{
	std::vector<Node*> t(0);
	t.push_back(_graph->at(nodeidA));
	addEdges(t, _graph->at(nodeidB), true);
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
			//std::stringstream buffer;
			//buffer << "dist " << dist << std::endl;
			//ROS_ERROR("%s", buffer.str().c_str());
			res.push_back((*iterator)->value);
		}

		



	}
	return res;		

}; 

int Roadmap::nodesInRange(rw::math::Q a)
{
	_kdnodesSearchResult.clear();
	int temp = NULL;
	double close = 10000000;

	
        _kdtree->nnSearchElipse(a, _radi ,_kdnodesSearchResult);
	for (std::list<const rwlibs::algorithms::KDTreeQ<Node*>::KDNode*>::const_iterator iterator = _kdnodesSearchResult.begin(), end = _kdnodesSearchResult.end(); iterator != end; ++iterator)
	{
		double dist = _metric->distance(a, (*iterator)->value->q_val);
		
		if(dist < close)
		{
			close = dist;
			temp = (*iterator)->value->nodenum;
		}

		



	}
	return temp;
	//return _kdtree->nnSearch(a).value->nodenum;		

}; 


void Roadmap::addEdges(std::vector<Node*> n, Node *a, bool check, std::vector<double> _cost)
{
	std::vector<Node*> addAble(0);
	
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
			if(!check)
			{
				n[i]->edges.push_back(a);
				n[i]->edge_cost.push_back(_cost[i]);
				a->edges.push_back(n[i]);
				a->edge_cost.push_back(_cost[i]);
				_connectedEdgePairs++;
				ROS_INFO("Here1");



			}
			else
			{
				if(SINGLE_THREAD)
				{
					if(!inCollision(a, n[i]))
					{
						double cost = _metric->distance(a->q_val, n[i]->q_val);
						n[i]->edges.push_back(a);
						n[i]->edge_cost.push_back(cost);
						a->edges.push_back(n[i]);
						a->edge_cost.push_back(cost);
						_connectedEdgePairs++;
						ROS_INFO("Here");
					}
				}
				else
				{
					addAble.push_back(n[i]);

				}
			}

		}	

	}
	

	if(!SINGLE_THREAD && check)
	{
		int i = 0;
		while( i< addAble.size())
		{
			if(threads.at(0) == nullptr)
			{
				threads.at(0) = new boost::thread(boost::bind(&Roadmap::threadAdd1, this, a, addAble[i]));
				//ROS_INFO("Thread 1 started");
				i++;
				t1Usage++;
				if(i >= addAble.size())
					break;

			}
			if(threads.at(1) == nullptr)
			{
				threads.at(1) = new boost::thread(boost::bind(&Roadmap::threadAdd2, this, a, addAble[i]));
				//ROS_INFO("Thread 2 started");
				i++;
				t2Usage++;
				if(i >= addAble.size())
					break;
			}
			if(threads.at(2) == nullptr)
			{
				threads.at(2) = new boost::thread(boost::bind(&Roadmap::threadAdd3, this, a, addAble[i]));
				//ROS_INFO("Thread 3 started");
				i++;
				t3Usage++;
				if(i >= addAble.size())
					break;
			}
			if(threads.at(3) == nullptr)
			{
				threads.at(3) = new boost::thread(boost::bind(&Roadmap::threadAdd4, this, a, addAble[i]));
				//ROS_INFO("Thread 4 started");
				i++;
				t4Usage++;
				if(i >= addAble.size())
					break;

			}
			for(int j = 0; j< threads.size(); j++)
			{
				if(threads.at(j) != nullptr && threads.at(j)->joinable())
				{
					//if(threads.at(j)->try_join_for(boost::chrono::milliseconds(1)))
					//{
						threads.at(j)->join();
						delete threads.at(j);
						threads.at(j) = nullptr;
					//}
				}

			}

			


		}
			// Last cleanup
			bool done = false;
			while(!done)
			{
				for(int j = 0; j< threads.size(); j++)
				{
					if(threads.at(j) != nullptr && threads.at(j)->joinable())
					{
						//if(threads.at(j)->try_join_for(boost::chrono::milliseconds(1)))
						//{
							threads.at(j)->join();
							delete threads.at(j);
							threads.at(j) = nullptr;
						//}
					}
				}

				if(threads.at(0) == nullptr && threads.at(1) == nullptr && threads.at(2) == nullptr && threads.at(3) == nullptr)
				{
					done = true; 
				}

			}

		
	}



};

void Roadmap::threadAdd1(Node* A, Node* B)
{
	if(!_edgeConstraint1->inCollision(A->q_val, B->q_val))
	{
  		boost::mutex::scoped_lock lock(push_lock);
		double cost = _metric->distance(A->q_val, B->q_val);
		B->edges.push_back(A);
		B->edge_cost.push_back(cost);
		A->edges.push_back(B);
		A->edge_cost.push_back(cost);
		_connectedEdgePairs++;
	}

}

void Roadmap::threadAdd2(Node* A, Node* B)
{
	if(!_edgeConstraint2->inCollision(A->q_val, B->q_val))
	{
  		boost::mutex::scoped_lock lock(push_lock);
		double cost = _metric->distance(A->q_val, B->q_val);
		B->edges.push_back(A);
		B->edge_cost.push_back(cost);
		A->edges.push_back(B);
		A->edge_cost.push_back(cost);
		_connectedEdgePairs++;
	}

}

void Roadmap::threadAdd3(Node* A, Node* B)
{
	if(!_edgeConstraint3->inCollision(A->q_val, B->q_val))
	{
  		boost::mutex::scoped_lock lock(push_lock);
		double cost = _metric->distance(A->q_val, B->q_val);
		B->edges.push_back(A);
		B->edge_cost.push_back(cost);
		A->edges.push_back(B);
		A->edge_cost.push_back(cost);
		_connectedEdgePairs++;
	}

}

void Roadmap::threadAdd4(Node* A, Node* B)
{
	if(!_edgeConstraint4->inCollision(A->q_val, B->q_val))
	{
  		boost::mutex::scoped_lock lock(push_lock);
		double cost = _metric->distance(A->q_val, B->q_val);
		B->edges.push_back(A);
		B->edge_cost.push_back(cost);
		A->edges.push_back(B);
		A->edge_cost.push_back(cost);
		_connectedEdgePairs++;
	}

}

void Roadmap::connectGraph()
{
	for(int i = 0; i< _graph->size(); i++)
	{
		std::stringstream checking;
		checking << "Checking Node: " << i+1 << " of " << _graph->size() <<  " for edges." << std::endl;
		ROS_INFO("%s", checking.str().c_str());
		std::vector<Node*> n = nodesInRange(_graph->at(i));
		addEdges(n, _graph->at(i), true);
	}


};

bool Roadmap::create_roadmap()
{
	while(_actualSize < _size)
	{
		std::stringstream creating;
		creating << "Creating Node: " << _actualSize +1 << " of " << _size << std::endl;
		ROS_INFO("%s", creating.str().c_str());
		for(int i = 0; i< 10000; i++)
			if(addNode())
				break;
			else if(i == 9999)
				return false;
		
	}
	
	connectGraph();
	int non = nonConnectedNodes();

	std::stringstream buffer0;
	buffer0 << "ThreadUsage: " << t1Usage << " " << t2Usage << " " << t3Usage << " " << t4Usage << std::endl;
	ROS_INFO("%s", buffer0.str().c_str());


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
	_workcell1 = rw::loaders::WorkCellLoader::Factory::load(path);
	_workcell2 = rw::loaders::WorkCellLoader::Factory::load(path);
	_workcell3 = rw::loaders::WorkCellLoader::Factory::load(path);
	_workcell4 = rw::loaders::WorkCellLoader::Factory::load(path);

	std::string path2 = ros::package::getPath("rovi2") + "/WorkStation_astar/WC3_Scene.wc.xml";
	_workcellAstar = rw::loaders::WorkCellLoader::Factory::load(path2);

	_device1 = _workcell1->findDevice("UR1");
	_device2 = _workcell2->findDevice("UR1");
	_device3 = _workcell3->findDevice("UR1");
	_device4 = _workcell4->findDevice("UR1");

	_deviceAstar = _workcellAstar->findDevice("UR1");

	_state1 =  _workcell1->getDefaultState();
	_state2 =  _workcell1->getDefaultState();
	_state3 =  _workcell1->getDefaultState();
	_state4 =  _workcell1->getDefaultState();
	
	_stateAstar = _workcellAstar->getDefaultState();


	// Create Graph
	_graph = new std::vector<Node*>(0);
	_graph->reserve(_size);


	// Create KdTree
	_kdtree = new rwlibs::algorithms::KDTreeQ<Node*>(_device1->getBounds().first.size());
	if(_kdtree == NULL)
		ROS_ERROR("KdTree error");


};


void Roadmap::initRobworkStuff()
{
	// Collision checking strategy for collision detection in workcell.
	_strategy1 = rwlibs::proximitystrategies::ProximityStrategyPQP::make();
	_strategy2 = rwlibs::proximitystrategies::ProximityStrategyPQP::make();
	_strategy3 = rwlibs::proximitystrategies::ProximityStrategyPQP::make();
	_strategy4 = rwlibs::proximitystrategies::ProximityStrategyPQP::make();
	_strategyAstar = rwlibs::proximitystrategies::ProximityStrategyPQP::make();
	if(_strategy1 == NULL)
		ROS_ERROR("Strategy error");


	// Detector for collision detection in workcell
	_detector1 = new rw::proximity::CollisionDetector(_workcell1, _strategy1);
	_detector2 = new rw::proximity::CollisionDetector(_workcell2, _strategy1);
	_detector3 = new rw::proximity::CollisionDetector(_workcell3, _strategy1);
	_detector4 = new rw::proximity::CollisionDetector(_workcell4, _strategy1);
	_detectorAstar = new rw::proximity::CollisionDetector(_workcellAstar, _strategyAstar);
	if(_detector1 == NULL)
		ROS_ERROR("Detector error");

	// Constraint, to check for workcell collision.
	_constraint1 = rw::pathplanning::QConstraint::make(_detector1, _device1, _state1);
	_constraint2 = rw::pathplanning::QConstraint::make(_detector2, _device2, _state2);
	_constraint3 = rw::pathplanning::QConstraint::make(_detector3, _device3, _state3);
	_constraint4 = rw::pathplanning::QConstraint::make(_detector4, _device4, _state4);
	_constraintAstar = rw::pathplanning::QConstraint::make(_detectorAstar, _deviceAstar, _stateAstar);

	// EdgeConstraint, to check for collision in an edge
	_edgeConstraint1 = rw::pathplanning::QEdgeConstraint::make(
            _constraint1, rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), _resolution);

	_edgeConstraint2 = rw::pathplanning::QEdgeConstraint::make(
            _constraint2, rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), _resolution);

	_edgeConstraint3 = rw::pathplanning::QEdgeConstraint::make(
            _constraint3, rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), _resolution);

	_edgeConstraint4 = rw::pathplanning::QEdgeConstraint::make(
            _constraint4, rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), _resolution);

	_edgeConstraintAstar = rw::pathplanning::QEdgeConstraint::make(
            _constraintAstar, rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), _resolution);

	// Make a uniform sampler, which only returns collision free samples for both bounds and workcell
	// We might not want a constrained sampler, if we wants to use our own bounds, alternative we can change the robots bounds!
	_sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device1),_constraint1);
	if(_sampler == NULL)
		ROS_ERROR("Sampler error");

	// Create metric weight, as motion weight in cartesian space.
	_metricWeights = rw::pathplanning::PlannerUtil::estimateMotionWeights(*_device1, _device1->getEnd(),_state1,rw::pathplanning::PlannerUtil::WORSTCASE,1000);

	// Create the metric
	_metric = rw::math::MetricFactory::makeWeightedEuclidean<rw::math::Q>(_metricWeights);
	//_metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	
	std::stringstream buffer;
	buffer << "Bounds +/- " << (_device1->getBounds().second)[0] << std::endl;
	ROS_INFO("%s", buffer.str().c_str());

	//_metricWeights = rw::math::Q(6,1,1,1,1,1,1);

	_radi = _metricWeights;

	std::stringstream buffer3;
	buffer3 << "Weights " << _metricWeights << std::endl;
	ROS_INFO("%s", buffer3.str().c_str());

        for(size_t i=0;i<_radi.size();i++){
            _radi[i] = _connection_radius/std::max(_metricWeights(i),0.1);
        }



        double nlen = _radi.norm2();
        for(size_t i=0;i<_radi.size();i++){
            _radi[i] = _radi[i]*(_connection_radius*std::sqrt(_device1->getBounds().first.size()))/nlen;
        }

	std::cout << "Radi: " << _radi << std::endl;
	_radi2 = _metricWeights;

        for(size_t i=0;i<_radi2.size();i++){
            _radi2[i] = _max_density/std::max(_metricWeights(i),0.1);
        }

        nlen = _radi2.norm2();
        for(size_t i=0;i<_radi2.size();i++){
            _radi2[i] = _radi2[i]*(_max_density*std::sqrt(_device1->getBounds().first.size()))/nlen;
        }
	std::cout << "Radi2: " << _radi2 << std::endl;

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
		/*for(int j = 0; j< connected_parts.at(i).size(); j++)
		{
			for(int k = 0; k < connected_parts.at(i).at(j)->edge_cost.size(); k++)
			{			
				std::stringstream ud;
				ud << "Cost " << connected_parts.at(i).at(j)->edge_cost.at(k) << std::endl;
				ROS_INFO("%s", ud.str().c_str());
			}

		}*/

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
				f << std::to_string(_graph->at(i)->edges.at(j)->nodenum) << '\t' << std::to_string(_graph->at(i)->edge_cost.at(j));
					
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
			std::vector<double> c(0);
			for(int i = 1; i< strs.size(); i = i + 2)
			{
				t.push_back(_graph->at(std::atoi(strs.at(i).c_str())));
				c.push_back(std::atof(strs.at(i+1).c_str()));
			}
			
			
			addEdges(t, _graph->at(std::atoi(strs.at(0).c_str())), false, c);			

		}

	}


}


int main(int argc, char **argv)
{
  time_t start, end;

  time(&start);

  ros::init(argc,argv,"roadmap31");
  ros::NodeHandle n;
  /*Roadmap Roadmap_ros(n, 1000, 0.01, 2.5, 0.7);
  if(Roadmap_ros.create_roadmap())
  {
	std::stringstream buffer;
	buffer << "Roadmap created with " << Roadmap_ros._actualSize << " Nodes and " << Roadmap_ros._connectedEdgePairs << " Edge pairs" << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
  }
  else
	ROS_INFO("Could not create Roadmap!");
  
  Roadmap_ros.connectedComponents();
  Roadmap_ros.save_roadmap("Roadmap_1000_0p01_2p5_0p7_connected.txt");

 
  time(&end);
  double dif = difftime(end, start);

  std::stringstream ti;
  ti << "Used time for creation in seconds: " << dif << std::endl; 
  ROS_INFO("%s", ti.str().c_str());
  





  */
   Roadmap Roadmap_ros(n, "Roadmap_500000_0p01_0p5_0p25_first/Roadmap_500000_0p01_0p5_0p25_first.txt");
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
