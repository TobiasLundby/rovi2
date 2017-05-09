#pragma once

// ROS
#include "ros/ros.h"
#include "rovi2/State.h"
#include "rovi2/position3D.h"
#include "rovi2/Q.h"

// Robwork
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

//General
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

class vision_test
{
public:
  vision_test(ros::NodeHandle);
  ~vision_test();

private:
  // NodeHandle
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber subscribe_robot_status;
  ros::Subscriber subscribe_triangulated;
  ros::Subscriber subscribe_estimated;
  ros::Subscriber subscribe_predicted;

  // Subscriber callbacks
  void robot_status_callback(const rovi2::State &msg);
  void triangulated_callback(const rovi2::position3D &msg);
  void estimated_callback(const rovi2::position3D &msg);
  void predicted_callback(const rovi2::position3D &msg);

  // Attributes
  rovi2::State robot_state;
  rovi2::position3D estimated_pos;
  rovi2::position3D triangulated_pos;

  // Robwork stuff
  rw::models::WorkCell::Ptr _workcell;
  rw::models::Device::Ptr _device;
  rw::kinematics::State _state;
  rw::kinematics::Frame* robot_base;
  rw::kinematics::Frame* TCP_marker;

  rw::math::Q toRw(const rovi2::Q& q);


};