// ROS
#include "ros/ros.h"

// General
#include "vision_test.hpp"

int main(int argc, char** argv)
{
  ROS_INFO("vision_test_node starting...");
  ros::init(argc, argv, "vision_test_node");
  ros::NodeHandle n;
  vision_test test_handle(n);

  ROS_INFO("vision_test_node ready...");

  while(ros::ok())
  {
    ros::spinOnce();
  }

}
