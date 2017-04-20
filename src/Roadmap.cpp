#include <rw/math/Q.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include "Roadmap.hpp"
#include <string>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/math/MetricFactory.hpp>


Roadmap::Roadmap(int size, double resolution, double connection_radius):

	_resolution(resolution),
	_size(size),
	_connection_radius(connection_radius)

{
	Roadmap::initWorkCell();
	std::stringstream buffer;
	buffer << "Default state: " << _device->getQ(_state) << std::endl;
	ROS_INFO("%s", buffer.str().c_str());
	

};

Roadmap::~Roadmap()
{
};

void Roadmap::initWorkCell()
{
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

};


int main(int argc, char **argv)
{
  ros::init(argc,argv,"roadmap");
  ros::NodeHandle n;
  Roadmap Roadmap_ros(1000, 0.01, 0.05);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
