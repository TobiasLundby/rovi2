#include "vision_test.hpp"

vision_test::vision_test(ros::NodeHandle n)
{
  nh = n;
  subscribe_robot_status = nh.subscribe("/rovi2/robot_node/Robot_state",0,&vision_test::robot_status_callback,this);
  subscribe_triangulated = nh.subscribe("/ball_locator_3d/pos_triangulated",0,&vision_test::triangulated_callback,this);
  subscribe_estimated = nh.subscribe("/ball_locator_3d/kalman_estimate",0,&vision_test::estimated_callback,this);
  subscribe_predicted = nh.subscribe("/ball_locator_3d/kalman_prediction",0,&vision_test::predicted_callback,this);


  _workcell = rw::loaders::WorkCellLoader::Factory::load("../WorkStation_3/WC3_Scene.wc.xml");
  _device = _workcell->findDevice("UR1");
  _state =  _workcell->getDefaultState();

  // Get frames
  rw::kinematics::Frame* robot_base = _workcell->findFrame("UR1.Base");
  rw::kinematics::Frame* TCP_marker = _workcell->findFrame("WSG50.TCPMarker");
  if(robot_base == NULL)
  {
    std::cout << "robot_base not found" << std::endl;
  }
  if(TCP_marker == NULL)
  {
    std::cout << "TCP_marker not found" << std::endl;
  }
  std::cout << "Setup complete" << std::endl;

}

vision_test::~vision_test()
{}

void vision_test::robot_status_callback(const rovi2::State &msg)
{
  robot_state = msg;
  ROS_INFO("Got a state");
}

void vision_test::triangulated_callback(const rovi2::position3D &msg)
{
  triangulated_pos = msg;
  ROS_INFO("Got a position");
}

void vision_test::estimated_callback(const rovi2::position3D &msg)
{
  estimated_pos = msg;
  ROS_INFO("Got an estimate");
}

void vision_test::predicted_callback(const rovi2::position3D &msg)
{

  ROS_INFO("Got a prediction");
}
