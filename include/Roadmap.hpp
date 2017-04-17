#ifndef ROADMAP_ROS
#define ROADMAP_ROS

#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/rw.hpp>

class Roadmap
{
public:

	Roadmap(); 

	virtual ~Roadmap();
	void initWorkCell();
 




protected:






public:
  	rw::models::WorkCell::Ptr _workcell;
	rw::kinematics::State _state;
  	rw::models::Device::Ptr _device;

};

#endif ROADMAP_ROS
