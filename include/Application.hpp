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
#include "rovi2/Q.h"
#include "rovi2/position3D.h"
#include "rovi2/State.h"
#include "rovi2/MovePtp.h"
#include <rw/invkin/JacobianIKSolver.hpp>

#include <list>


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


	void RobotCallback(const rovi2::State &State);
	void GoalCallback(const rovi2::position3D &position);
	void PathCallback(const rovi2::path &path_in);




protected:
	void initWorkCell();

	

	
private:
  	rw::models::WorkCell::Ptr _workcell;
	rw::kinematics::State _state;
  	rw::models::Device::Ptr _device;
	rw::math::Q _metricWeights;
	rw::math::QMetric::Ptr _metric;

	bool plan_incoming = false;
	bool starting = true;


	rw::invkin::JacobianIKSolver *_solver; 
	rw::kinematics::Frame *_BallErrorFrame;

	ros::Subscriber robot_controller;
	ros::Subscriber goal_controller;
	ros::Subscriber path_controller;
	ros::ServiceClient robot_call;
	ros::ServiceClient path_call;


	std::list<rw::math::Q> path;
	std::list<rw::math::Q> running_path;
	rw::math::Q moving_to;

	ros::NodeHandle _n;
	

};

#endif
