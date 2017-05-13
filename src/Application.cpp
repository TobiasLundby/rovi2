#include <rw/math/Q.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include "Application.hpp"
#include <string>
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



Application::Application(ros::NodeHandle h):

	_nodehandle(h)

{

	initWorkCell();
	

};


Application::~Application()
{

	delete _workcell;
  	delete _device;
	delete _metric;
};

/************************************************************************
 * Q  -> lend from Caros
 ************************************************************************/
rw::math::Q Application::toRw(const rovi2::Q& q)
{
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

rovi2::Q Application::toRos(const rw::math::Q& q)
{
  rovi2::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}


void Application::initWorkCell()
{
	std::string path = ros::package::getPath("rovi2") + "/WorkStation_astar/WC3_Scene.wc.xml";
	_workcell = rw::loaders::WorkCellLoader::Factory::load(path);

	_device = _workcell->findDevice("UR1");

	_state =  _workcell->getDefaultState();


// Create metric weight, as motion weight in cartesian space.
	_metricWeights = rw::pathplanning::PlannerUtil::estimateMotionWeights(*_device, _device->getEnd(),_state,rw::pathplanning::PlannerUtil::WORSTCASE,1000);

	// Create the metric
	_metric = rw::math::MetricFactory::makeWeightedEuclidean<rw::math::Q>(_metricWeights);


};



int main(int argc, char **argv)
{

  ros::init(argc,argv,"Application_node");
  ros::NodeHandle n;
  
   Application Application_node(n);


  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
