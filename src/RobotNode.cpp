#include "RobotNode.h"



RobotNode_ros::RobotNode_ros(ros::NodeHandle h)
{

	_nodehandle = h;

	RobotNode_ros::initWorkCell();
	RobotNode_ros::initDevice();
	service_nonlinear = _nodehandle.advertiseService("rovi2/robot_node/Move_nonlinear_ptp", &RobotNode_ros::Move_nonlinear_ptp, this);
        state_updater = _nodehandle.advertise<rovi2::State>("rovi2/robot_node/Robot_state", 1000);
};

RobotNode_ros::~RobotNode_ros()
{
};

/************************************************************************
 * Q  -> lend from Caros
 ************************************************************************/
rw::math::Q RobotNode_ros::toRw(const rovi2::Q& q)
{
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

rovi2::Q RobotNode_ros::toRos(const rw::math::Q& q)
{
  rovi2::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}

/*****************************************************************/


void RobotNode_ros::initWorkCell()
{
    ros::NodeHandle node("~");
    std::string workcell_file;
    bool found_param;
    found_param = node.getParam("/sample/workcell", workcell_file);
    if (!found_param)
    {
      ROS_ERROR_STREAM("No such parameter on the parameter server: " << "/sample/workcell");
    }
	_workcell = rw::loaders::WorkCellLoader::Factory::load(workcell_file);

	_device = _workcell->findDevice("UR1");

	_state =  _workcell->getDefaultState();


};


void RobotNode_ros::initDevice()
{
	_ur = new rwhw::URCallBackInterface;
	_urrt = new rwhw::UniversalRobotsRTLogging;
	  try
	  {
	    	_urrt->connect(ROBOT_ip, 30003);
	  }
	  catch (rw::common::Exception& exp)
	  {
		ROS_ERROR_STREAM("Could not connect to urrt");
	  }

	  try
	  {
	    	_ur->connect(ROBOT_ip, 30002);
	  }
	  catch (rw::common::Exception& exp)
	  {
		ROS_ERROR_STREAM("Could not connect to urrt");
	  }


	_ur->startCommunication(33334);
	_urrt->start();

	publisher = boost::thread(&RobotNode_ros::StatePublisher, this);



}

rw::math::Q RobotNode_ros::getQ()
{


}

void RobotNode_ros::StatePublisher()
{
	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		if (_urrt->hasData() == false || _ur->getPrimaryInterface().hasData() == false)
		{
			continue;
		}

		rwhw::URRTData urrt_data = _urrt->getLastData();
		rwhw::UniversalRobotsData ur_data = _ur->getPrimaryInterface().getLastData();
		_ur->getPrimaryInterface().clearMessages();
		rovi2::State current_state;
		bool data_valid = false;
		if (urrt_data.qActual.size() == 6)
		{
			data_valid = true;
			qCurrent = urrt_data.qActual;
			current_state.q = RobotNode_ros::toRos(urrt_data.qActual);
			current_state.dq = RobotNode_ros::toRos(urrt_data.dqActual);
			bool is_moving = false;
			for(int i = 0; i< 6; i++)
			{
				if(fabs(urrt_data.dqActual[i]) > 0.0001)
				{
					is_moving = true;
					break;
				}

			}
			current_state.is_moving = is_moving;
			current_state.is_moving2 = _ur->isMoving();
		}
		current_state.valid = data_valid;
	    	current_state.e = ur_data.emergencyStopped;

		state_updater.publish(current_state);
		loop_rate.sleep();
	}


}


bool RobotNode_ros::Move_nonlinear_ptp(rovi2::MovePtp::Request & request, rovi2::MovePtp::Response &res)
{
	//TODO check for bounds!
	rw::math::Q newQ = RobotNode_ros::toRw(request.target);
	_ur->moveQ(newQ, 1.0);
	res.success = true;

	return true;
	
}