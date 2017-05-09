#include "ros/ros.h"
#include "rovi2/xyz.h"
#include "rovi2/Movexyz.h"

//#include <rw/rw.hpp>
//#include <rw/math.hpp>
//#include <rw/math/Vector3D.hpp>
//#include <rw/math/Transform3D.hpp>
//#include <rw/kinematics/Frame.hpp>
//#include <rw/models/WorkCell.hpp>
//#include <rw/kinematics/State.hpp>
//#include <rw/models/Device.hpp>
//#include <rw/math/Q.hpp>
//#include <boost/algorithm/string.hpp>
//#include <rw/math/RPY.hpp>

int main(int argc, char** argv)
{
  ROS_INFO("vision_test_robot_node starting...");
  ros::init(argc, argv, "vision_test_robot_node");
  ros::NodeHandle n;

  rovi2::Movexyz service_call;
  //rovi2::xyz;
  ROS_INFO("#1");
  service_call.request.target.data.push_back(-0.4);//[0] = -0.4;
  service_call.request.target.data.push_back(-0.4); //[1] = -0.4;
  service_call.request.target.data.push_back(0.6);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  for(int i = 0; i < 6; i++)
    ROS_INFO("%f",service_call.request.target.data[i]);
  ros::service::call("/rovi2/robot_node/MoveXYZ",service_call);
  ROS_INFO("Response: %d",service_call.response.success);
  //ros::Duration(20).sleep();
  ros::spinOnce();
  ROS_INFO("#2");
  service_call.request.target.data.clear();
  service_call.request.target.data.push_back(-0.4);//[0] = -0.4;
  service_call.request.target.data.push_back(-0.4); //[1] = -0.4;
  service_call.request.target.data.push_back(0.7);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  ros::service::call("/rovi2/robot_node/MoveXYZ",service_call);
  ROS_INFO("Response: %d",service_call.response.success);

  ros::Duration(2).sleep();
  ros::spinOnce();
  ROS_INFO("#3");
  ros::Duration(2).sleep();
  ros::spinOnce();
  ROS_INFO("#4");
  ros::Duration(2).sleep();
  ros::spinOnce();
}
