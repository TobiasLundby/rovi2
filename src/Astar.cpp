#include <algorithm>
#include "ros/ros.h"
#include <functional>
#include <queue>
#include "Astar.hpp"
#include "Roadmap.hpp"
#include <limits>


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

	_openList = NULL;
	_numRun = 0;

}



Astar::~Astar()
{
	if(!_openList == NULL)
		delete _openList;

}

double Astar::calc_h(int cId, int gId)
{
	return _metric->distance(_graph->at(cId)->q_val, _graph->at(gId)->q_val);

}


void Astar::find_path(int startNodeId, int goalNodeId, std::vector<int> &path)
{
	_numRun++;
	if(_openList == NULL)
			_openList = new std::priority_queue<Node*, std::vector<Node*>, sort_func>;


	// Init start Node (Root)
	_graph->at(startNodeId)->g_score = 0;
	_graph->at(startNodeId)->f_score = calc_h(startNodeId, goalNodeId);
	_graph->at(startNodeId)->astar_run = _numRun;
	_graph->at(startNodeId)->closed = false;
	_graph->at(startNodeId)->open = true;

	while(!_openList->empty())
	{
		Node* current = _openList->top();
		_openList->pop();
		if(current->nodenum == goalNodeId)
		{
			ROS_INFO("Found path");

			delete _openList;
			_openList = NULL;

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
	
	ROS_INFO("No path found!");

	delete _openList;
	_openList = NULL;
}
