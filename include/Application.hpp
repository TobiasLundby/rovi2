#ifndef APPLICATION_ROS
#define APPLICATION_ROS

#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include "rovi2/Q.h"
#include "rovi2/Plan.h"
#include "rovi2/path.h"
#include "rovi2/Conf.h"
#include <rw/common.hpp>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
//#include <boost/thread/lock_guard.hpp>
#include <boost/bind.hpp>





class Application
{
public:

	Application(ros::NodeHandle h); 

	virtual ~Application();



	rw::math::Q toRw(const rovi2::Q& q);
	rovi2::Q toRos(const rw::math::Q& q);

	 
 




protected:
	void initWorkCell();

	

	
public:
  	rw::models::WorkCell::Ptr _workcell;
	rw::kinematics::State _state;
  	rw::models::Device::Ptr _device;
	rw::math::Q _metricWeights;
	rw::math::QMetric::Ptr _metric;


	boost::mutex push_lock;
	boost::thread* astar_thread = nullptr;
	std::vector<boost::thread*> threads;

	ros::NodeHandle _nodehandle;
	

};

#endif
