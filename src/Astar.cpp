#include <algorithm>
#include "ros/ros.h"
#include <functional>
#include <queue>
#include "Astar.hpp"
#include "Roadmap.hpp"
#include <limits>

#define NO_COLLISION_CHECK true


struct sort_func {
    bool operator()(Node* a, Node* b)
    {
        return a->f_score > b->f_score;
    }
};

Astar::Astar(double size, std::vector<Node*> *graph, 	rw::math::QMetric::Ptr metric, rw::common::Ptr<rw::pathplanning::QConstraint> constraint, 	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> edgeConstraint)

	: _size(size),
	_graph(graph),
	_metric(metric),
	_constraint(constraint),
	_edgeConstraint(edgeConstraint)
{

	_openList = nullptr;
	_numRun = 0;

}



Astar::~Astar()
{
	if(!(_openList == nullptr))
		delete _openList;

}


/************************************************************************
 * Q  -> lend from Caros
 ************************************************************************/

rovi2::Q Astar::toRos(const rw::math::Q& q)
{
  rovi2::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}

double Astar::calc_h(int cId, int gId)
{
	return _metric->distance(_graph->at(cId)->q_val, _graph->at(gId)->q_val);

}


void Astar::find_path(int startNodeId, int goalNodeId, rovi2::path &path)
{
	std::stringstream buffer;
	buffer << "startNodeId: " << startNodeId << " goalNodeId: " << goalNodeId << std::endl;
	ROS_INFO("%s", buffer.str().c_str());


	ROS_INFO("Im here 8");
	_numRun++;
	if(_openList == nullptr)
			_openList = new std::priority_queue<Node*, std::vector<Node*>, sort_func>;


	// Init start Node (Root)
	_graph->at(startNodeId)->g_score = 0;
	_graph->at(startNodeId)->f_score = calc_h(startNodeId, goalNodeId);
	_graph->at(startNodeId)->astar_run = _numRun;
	_graph->at(startNodeId)->closed = false;
	_graph->at(startNodeId)->open = true;
	
	_openList->push(_graph->at(startNodeId));
	while(!_openList->empty())
	{
		Node* current = _openList->top();
		_openList->pop();
		if(NO_COLLISION_CHECK )//|| !_constraint->inCollision(current->q_val))
		{
			if(current->nodenum == goalNodeId)
			{
				ROS_INFO("Found path");
				std::vector<int> path_temp(0);
				std::stringstream buffer;
				buffer << "Final goal position " << _graph->at(current->nodenum)->q_val << std::endl;
				ROS_INFO("%s", buffer.str().c_str());
				while(current->nodenum != startNodeId)
				{	
					//ROS_INFO("Test");
					path_temp.push_back(current->nodenum);
					current = current->cameFrom;

				}

				for(int i = 0; i< path_temp.size(); i++)
				{
					if(i < 10)
					{
						path.data.push_back(toRos(_graph->at(path_temp.at(path_temp.size()-1-i))->q_val));

					}
					else
						break;


				}


				//path.data.push_back(current->nodenum);
				delete _openList;
				_openList = nullptr;

				return;

			}
			current->open = false;
			current->closed = true;
		

			for(int i = 0; i< current->edges.size(); i++)
			{
				if(current->edges.at(i)->astar_run != _numRun)
				{
					current->edges.at(i)->astar_run = _numRun;
					current->edges.at(i)->closed = false;
					current->edges.at(i)->open = false;
					current->edges.at(i)->g_score = std::numeric_limits<double>::max();
					current->edges.at(i)->f_score = std::numeric_limits<double>::max();
				}

				if(current->edges.at(i)->closed == false)	
				{
					double _g_score = current->g_score + current->edge_cost.at(i);
					double _f_score = current->edges.at(i)->g_score + calc_h(current->edges.at(i)->nodenum, goalNodeId);
					if(current->edges.at(i)->open == false)
					{	
						current->edges.at(i)->open = true;
						_openList->push(current->edges.at(i));
						current->edges.at(i)->cameFrom = current;

					}
				
					if(current->edges.at(i)->f_score > _f_score)
					{
						current->edges.at(i)->cameFrom = current;
						current->edges.at(i)->g_score = _g_score;		
						current->edges.at(i)->f_score = _f_score;
					}
				}
			}
		}
		else
		{
			current->open = false;
			current->closed = true;

		}
	}
	
	ROS_INFO("No path found!");

	delete _openList;
	_openList = nullptr;
}
