#include "vision_test.hpp"

vision_test::vision_test(ros::NodeHandle n)
{
  nh = n;
  subscribe_robot_status = nh.subscribe("/rovi2/robot_node/Robot_state",0,&vision_test::robot_status_callback,this);
  subscribe_triangulated = nh.subscribe("/ball_locator_3d/pos_triangulated",0,&vision_test::triangulated_callback,this);
  subscribe_estimated = nh.subscribe("/ball_locator_3d/kalman_estimate",0,&vision_test::estimated_callback,this);
  subscribe_predicted = nh.subscribe("/ball_locator_3d/kalman_prediction",0,&vision_test::predicted_callback,this);
  subscribe_xy_left = nh.subscribe("/ball_locator_3d/pos_left",0,&vision_test::xy_left_callback,this);
  subscribe_xy_right = nh.subscribe("/ball_locator_3d/pos_right",0,&vision_test::xy_right_callback,this);

  _workcell = rw::loaders::WorkCellLoader::Factory::load("/home/nikolaj/catkin_ws_A/src/rovi2/WorkStation_3/WC3_Scene.wc.xml");
  _device = _workcell->findDevice("UR1");
  _state =  _workcell->getDefaultState();

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
  std::cout << "Setup complete" << std::endl;

}

vision_test::~vision_test()
{}

void vision_test::robot_status_callback(const rovi2::State &msg)
{
  robot_state = msg;

 rw::math::Q q = toRw(robot_state.q);
  _device->setQ(q,_state);
  rw::math::Vector3D<double> robot_p = robot_base->fTf(TCP_marker,_state).P();


  // Transformation matrices
  rw::math::Vector3D<double> transP(-0.116075, -1.67057, 1.33754);
  rw::math::Rotation3D<double> transR(rw::math::RPY<double>(-0.0375525, -0.0132076, -2.0942).toRotation3D());
  rw::math::Transform3D<double> trans(transP, transR);
//Transform camera points to robot points
  //rw::math::Vector3D<double> triangulated_pos_robot(triangulated_pos.x, triangulated_pos.y, triangulated_pos.z);
  //triangulated_pos_robot = trans*triangulated_pos_robot;
  //rw::math::Vector3D<double> estimated_pos_robot(estimated_pos.x, estimated_pos.y, estimated_pos.z);
  //estimated_pos_robot = trans*estimated_pos_robot;
  rw::math::Vector3D<double> predicted_pos_robot(msg.x,msg.y,msg.z);
  predicted_pos_robot = trans*predicted_pos_robot;

  std::ofstream file;
  file.open("vision_test.log",std::ios::app);
  file << predicted_pos_robot(0) << "\t";
  file << predicted_pos_robot(1) << "\t";
  file << predicted_pos_robot(2) << "\t";
  file << robot_p(0) << "\t";
  file << robot_p(1) << "\t";
  file << robot_p(2) << "\t";
  file << q(0) << "\t";
file << q(1) << "\t";
file << q(2) << "\t";
file << q(3) << "\t";
file << q(4) << "\t";
file << q(5) << "\t";
  file << std::endl;
  file.close();
  //ROS_INFO("Got a state");
}

void vision_test::triangulated_callback(const rovi2::position3D &msg)
{
  triangulated_pos = msg;
  //ROS_INFO("Triangulated: %f\t%f\t%f", msg.x, msg.y msg.z);
}

void vision_test::estimated_callback(const rovi2::position3D &msg)
{
  estimated_pos = msg;
  //ROS_INFO("Got an estimate");
}

void vision_test::xy_left_callback(const rovi2::position2D &msg)
{
  xy_left_position = msg;
}

void vision_test::xy_right_callback(const rovi2::position2D &msg)
{
  xy_right_position = msg;
}

void vision_test::predicted_callback(const rovi2::position3D &msg)
{

	predicted_pos = msg;
  /*// Convert ros q to rw q, set state and conver to cartesian space (forward kinematics)
  rw::math::Q q = toRw(robot_state.q);
  _device->setQ(q,_state);
  rw::math::Vector3D<double> robot_p = robot_base->fTf(TCP_marker,_state).P();

  // Transformation matrices
  rw::math::Vector3D<double> transP(-0.116075, -1.67057, 1.33754);
  rw::math::Rotation3D<double> transR(rw::math::RPY<double>(-0.0375525, -0.0132076, -2.0942).toRotation3D());
  rw::math::Transform3D<double> trans(transP, transR);

  //Transform camera points to robot points
  rw::math::Vector3D<double> triangulated_pos_robot(triangulated_pos.x, triangulated_pos.y, triangulated_pos.z);
  triangulated_pos_robot = trans*triangulated_pos_robot;
  rw::math::Vector3D<double> estimated_pos_robot(estimated_pos.x, estimated_pos.y, estimated_pos.z);
  estimated_pos_robot = trans*estimated_pos_robot;
  rw::math::Vector3D<double> predicted_pos_robot(msg.x,msg.y,msg.z);
  predicted_pos_robot = trans*predicted_pos_robot;

  std::ofstream file;
  file.open("vision_test.log",std::ios::app);
  file << triangulated_pos_robot(0) << "\t";
  file << triangulated_pos_robot(1) << "\t";
  file << triangulated_pos_robot(2) << "\t";
  file << estimated_pos_robot(0) << "\t";
  file << estimated_pos_robot(1) << "\t";
  file << estimated_pos_robot(2) << "\t";
  file << predicted_pos_robot(0) << "\t";
  file << predicted_pos_robot(1) << "\t";
  file << predicted_pos_robot(2) << "\t";
  file << robot_p(0) << "\t";
  file << robot_p(1) << "\t";
  file << robot_p(2) << "\t";
  file << xy_left_position.x << "\t";
  file << xy_left_position.y << "\t";
  file << xy_right_position.x << "\t";
  file << xy_right_position.y << "\t";
  file << std::endl;
  file.close();
	*/
  //ROS_INFO("Got a prediction");
}

rw::math::Q vision_test::toRw(const rovi2::Q& q)
{ // From RobWork
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}
