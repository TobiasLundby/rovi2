#ifndef ROADMAP_ROS
#define ROADMAP_ROS

#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include "rovi2/Q.h"
#include "rovi2/Plan.h"
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

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
//#include <boost/thread/lock_guard.hpp>
#include <boost/bind.hpp>

#define SINGLE_THREAD false


class Astar;




class Node
{
public:
 	Node(rw::math::Q p, int nodeid){ q_val = p; edges = std::vector<Node*>(0); edge_cost = std::vector<double>(0); nodenum = nodeid;}
	~Node();
	 

	rw::math::Q q_val;
	std::vector<Node*> edges;
	std::vector<double> edge_cost;
        bool connected_component = false;
        bool usable = false;
	int nodenum;

	unsigned int astar_run = 0;
	double g_score;
	double f_score;
	bool closed;
	bool open;
	Node* cameFrom;
};




class Roadmap
{
public:

	Roadmap(ros::NodeHandle h, int size, double resolution, double connection_radius, double max_density); 
	Roadmap(ros::NodeHandle h,std::string path);

	virtual ~Roadmap();

        bool create_roadmap();
        void save_roadmap(std::string path);


	rw::math::Q toRw(const rovi2::Q& q);


	
	// ROS topics	

	// We probably need to implement the graph search in here, givin the transfer time for ros topics will slow it down
	void find_path(rw::math::Q init, rw::math::Q goal);

	// If the distance is small enough, do not start a graph search, insted make graping using RRT (seperate ROS topic!)
	// Always check with this one before calling find_path
	void distance_to_goal(rw::math::Q init, rw::math::Q goal);

	void connectedComponents();


	bool start_plan(rovi2::Plan::Request & request, rovi2::Plan::Response &res);
	 
 




protected:
	void initWorkCell();
	void initRobworkStuff();

	

	// Find connection node for init and goal from a Q position
	Node* find_connection_node(rw::math::Q);
	bool inCollision(Node *n);
	bool inCollision(Node *a, Node *b);

	void threadAdd1(Node* A, Node* B);
	void threadAdd2(Node* A, Node* B);
	void threadAdd3(Node* A, Node* B);
	void threadAdd4(Node* A, Node* B);

	bool distanceTooClose(rw::math::Q a);

	// These are not checked for collision!
	void addNode(rw::math::Q n, int nodeid, bool c, bool u);
	void addNode(rw::math::Q n, int nodeid);
	void addEdges(int nodeidA, int nodeidB);

	// Add a new samples node
	bool addNode();

	std::vector<Node*> nodesInRange(Node *a); 
	void addEdges(std::vector<Node*> n, Node *a, bool check, std::vector<double> _cost = std::vector<double>(0));
	void connectGraph();

	int nonConnectedNodes();

	void load_roadmap(std::string path);

	static bool Sorting(const std::vector<Node*> i, const std::vector<Node*> j) { return i.size() > j.size(); }









public:
  	rw::models::WorkCell::Ptr _workcell1 = nullptr;
  	rw::models::WorkCell::Ptr _workcell2 = nullptr;
  	rw::models::WorkCell::Ptr _workcell3 = nullptr;
  	rw::models::WorkCell::Ptr _workcell4 = nullptr;
	rw::models::WorkCell::Ptr _workcellAstar = nullptr;
	rw::kinematics::State _state1;
	rw::kinematics::State _state2;
	rw::kinematics::State _state3;
	rw::kinematics::State _state4;
	rw::kinematics::State _stateAstar;
  	rw::models::Device::Ptr _device1;
  	rw::models::Device::Ptr _device2;
  	rw::models::Device::Ptr _device3;
  	rw::models::Device::Ptr _device4;
	rw::models::Device::Ptr _deviceAstar;
	rw::proximity::CollisionDetector::Ptr _detector1;
	rw::proximity::CollisionDetector::Ptr _detector2;
	rw::proximity::CollisionDetector::Ptr _detector3;
	rw::proximity::CollisionDetector::Ptr _detector4;
	rw::proximity::CollisionDetector::Ptr _detectorAstar;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint1;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint2;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint3;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint4;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraintAstar;
	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edgeConstraint1;
	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edgeConstraint2;
	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edgeConstraint3;
	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edgeConstraint4;
	rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edgeConstraintAstar;
	rw::proximity::CollisionStrategy::Ptr _strategy1;
	rw::proximity::CollisionStrategy::Ptr _strategy2;
	rw::proximity::CollisionStrategy::Ptr _strategy3;
	rw::proximity::CollisionStrategy::Ptr _strategy4;
	rw::proximity::CollisionStrategy::Ptr _strategyAstar;
	rw::pathplanning::QSampler::Ptr _sampler;
	rw::math::Q _metricWeights;
	rw::math::QMetric::Ptr _metric = nullptr;
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
	double _usable_nodes;
	double _largestConnected = 0;

	int t1Usage = 0;
	int t2Usage = 0;
	int t3Usage = 0;
	int t4Usage = 0;

	boost::mutex push_lock;
	//boost::mutex astar_lock;
	//bool astar_running = false;
	Astar *planner;
	boost::thread* astar_thread = nullptr;
	std::vector<boost::thread*> threads;

	ros::ServiceServer service_start_plan;
	ros::NodeHandle _nodehandle;
	

};

#endif
