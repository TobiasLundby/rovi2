#include <rw/math/Q.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include "Roadmap.hpp"
#include <string>


Roadmap::Roadmap()
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

};


int main(int argc, char **argv)
{
  ros::init(argc,argv,"roadmap");
  ros::NodeHandle n;
  Roadmap Roadmap_ros;
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
