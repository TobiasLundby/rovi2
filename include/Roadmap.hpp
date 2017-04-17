#ifndef ROADMAP_ROS
#define ROADMAP_ROS

#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>

class Node
{
 	Node(rw::math::Q p){ q_val = p; edges = std::vector<Node*>(0);}
	~Node();
	 
	void add_edge(Node* e){ edges.push_back(e);}

	rw::math::Q q_val;
	std::vector<Node*> edges;
        bool connected = false;
};


class Roadmap
{
public:

	Roadmap(); 

	virtual ~Roadmap();

        void create_roadmap(int size, double resolution, double connection_radius);
        void save_roadmap(std::string path);
	void load_roadmap(std::string path);

	
	// ROS topics	

	// We probably need to implement the graph search in here, givin the transfer time for ros topics will slow it down
	void find_path(rw::math::Q init, rw::math::Q goal);

	// If the distance is small enough, do not start a graph search, insted make graping using RRT (seperate ROS topic!)
	void distance_to_goal(rw::math::Q init, rw::math::Q goal);
	 
 




protected:
	void initWorkCell();

	// Find connection node for init and goal from a Q position
	Node* find_connection_node(rw::math::Q);






public:
  	rw::models::WorkCell::Ptr _workcell;
	rw::kinematics::State _state;
  	rw::models::Device::Ptr _device;
        std::vector<Node*> *_graph;

};

#endif ROADMAP_ROS
