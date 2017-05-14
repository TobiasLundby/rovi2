#include "ros/ros.h"
#include "rovi2/xyz.h"
#include "rovi2/Movexyz.h"

#include <rw/rw.hpp>
#include <rw/math.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/math/Q.hpp>
#include <boost/algorithm/string.hpp>
#include <rw/math/RPY.hpp>
#include <sstream>

int main(int argc, char** argv)
{
  ROS_INFO("vision_test_robot_node starting...");
  ros::init(argc, argv, "vision_test_robot_node");
  ros::NodeHandle n;

  rw::math::Vector3D<double> transP(-0.116075, -1.67057, 1.33754);
  rw::math::Rotation3D<double> transR(rw::math::RPY<double>(-0.0375525, -0.0132076, -2.0942).toRotation3D());
  rw::math::Transform3D<double> trans(transP, transR);

  rw::kinematics::Frame* robot_base;
  rw::kinematics::Frame* TCP_marker;

  rw::models::WorkCell::Ptr _workcell = rw::loaders::WorkCellLoader::Factory::load("/home/mathias/catkin_ws/src/rovi2/WorkStation_3/WC3_Scene.wc.xml");
  rw::models::Device::Ptr _device = _workcell->findDevice("UR1");
  rw::kinematics::State _state =  _workcell->getDefaultState();

  // Get frames
  robot_base = _workcell->findFrame("UR1.Base");
  TCP_marker = _workcell->findFrame("WSG50.BallError");
  if(robot_base == NULL)
  {
    std::cout << "robot_base not found" << std::endl;
  }
  if(TCP_marker == NULL)
  {
    std::cout << "TCP_marker not found" << std::endl;
  }

  std::stringstream buf;
  buf << *(robot_base) << " , " << *(TCP_marker) << std::endl;
  ROS_INFO("%s",buf.str().c_str());
  //rw::math::Q q(6,-0.4588, -1.7026, -0.9179, -0.5139, 0.4561, 0.0188);
  rw::math::Q q(6,-0.45886514427998293, -1.7026772253206577, -0.9179848271019003, -0.513925856428815, 0.45612188890540106, 0.0188171385940185);
  _device->setQ(q,_state);
  rw::math::Vector3D<double> robot_p = robot_base->fTf(TCP_marker,_state).P();
  ROS_INFO("Robot start:");
  buf.str("");
  buf << "Position: "<< robot_p(0) << " " << robot_p(1) << " " << robot_p(2);
  ROS_INFO("%s",buf.str().c_str());
  ROS_INFO("%d",robot_p(0));
  ROS_INFO("%d",robot_p(1));
  ROS_INFO("%d",robot_p(2));

  rovi2::Movexyz service_call;
  //rovi2::xyz;
  ROS_INFO("#1");
  rw::math::Vector3D<double> pos_robot(-0.26, -0.13, 1.22);
  pos_robot = trans*pos_robot;

  service_call.request.target.data.push_back(robot_p(0)+0.02);//-0.4);//[0] = -0.4;
  service_call.request.target.data.push_back(robot_p(1));//-0.4); //[1] = -0.4;
  service_call.request.target.data.push_back(robot_p(2));//0.6);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  for(int i = 0; i < 6; i++)
    ROS_INFO("%f",service_call.request.target.data[i]);
  ros::service::call("/rovi2/robot_node/MoveXYZ",service_call);
  ROS_INFO("Response: %d",service_call.response.success);
  //ros::Duration(20).sleep();
  ros::spinOnce();
/*  ROS_INFO("#2");
  service_call.request.target.data.clear();
  service_call.request.target.data.push_back(-0.4);//[0] = -0.4;
  service_call.request.target.data.push_back(-0.4); //[1] = -0.4;
  service_call.request.target.data.push_back(0.7);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  service_call.request.target.data.push_back(0);
  ros::service::call("/rovi2/robot_node/MoveXYZ",service_call);
  ROS_INFO("Response: %d",service_call.response.success);
*/
  ros::Duration(2).sleep();
  ros::spinOnce();
  ROS_INFO("#3");
  ros::Duration(2).sleep();
  ros::spinOnce();
  ROS_INFO("#4");
  ros::Duration(2).sleep();
  ros::spinOnce();
}
