#ifndef ROADMAP_ROS
#define ROADMAP_ROS

#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
//#include <rw/math/QMetric.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QEdgeConstraint.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/common.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

bool Sorting(int i, int j) { return i > j; }


class Node
{
public:
 	Node(rw::math::Q p, int nodeid){ q_val = p; edges = std::vector<Node*>(0); nodenum = nodeid;}
	~Node();
	 
	void add_edge(Node* e){ edges.push_back(e);}

	rw::math::Q q_val;
	std::vector<Node*> edges;
        bool connected = false;
        bool connected_component = false;
	int nodenum;
};


class Roadmap
{
public:

	Roadmap(int size, double resolution, double connection_radius, double max_density); 

	virtual ~Roadmap();

        bool create_roadmap();
        void save_roadmap(std::string path);
	void load_roadmap(std::string path);

	
	// ROS topics	

	// We probably need to implement the graph search in here, givin the transfer time for ros topics will slow it down
	void find_path(rw::math::Q init, rw::math::Q goal);

	// If the distance is small enough, do not start a graph search, insted make graping using RRT (seperate ROS topic!)
	// Always check with this one before calling find_path
	void distance_to_goal(rw::math::Q init, rw::math::Q goal);

	void connectedComponents();
	 
 




protected:
	void initWorkCell();

	// Find connection node for init and goal from a Q position
	Node* find_connection_node(rw::math::Q);
	bool inCollision(Node *n);
	bool inCollision(Node *a, Node *b);

	bool distanceTooClose(rw::math::Q a);

	// These are not checked for collision!
	void addNode(rw::math::Q, int);
	void addEdges(int nodeidA, int nodeidB);

	// Add a new samples node
	bool addNode();

	std::vector<Node*> nodesInRange(Node *a); 
	void addEdges(std::vector<Node*> n, Node *a);
	void connectGraph();

	int nonConnectedNodes();









public:
  	rw::models::WorkCell::Ptr _workcell;
	rw::kinematics::State _state;
  	rw::models::Device::Ptr _device;
	rw::proximity::CollisionDetector::Ptr _detector;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint;
	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edgeConstraint;
	rw::proximity::CollisionStrategy::Ptr _strategy;
	rw::pathplanning::QSampler::Ptr _sampler;
	rw::math::Q _metricWeights;
	rw::math::QMetric::Ptr _metric;
	rw::math::Q _radi;
	rw::math::Q _radi2;

	// KdTree for nearest neighbor search.
        rwlibs::algorithms::KDTreeQ<Node*>::Ptr  _kdtree;
        std::list<const rwlibs::algorithms::KDTreeQ<Node*>::KDNode*> _kdnodesSearchResult;

	// The graph is just a container for the node pointers.
	std::vector<Node*> *_graph;

	double _resolution;
	double _size;
	double _actualSize = 0;
	double _connectedEdgePairs = 0;
	double _connection_radius;
	double _max_density;
	

};

#endif
