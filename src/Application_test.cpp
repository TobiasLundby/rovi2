#include <rw/math/Q.hpp>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include "Application.hpp"
#include <string>
#include <rw/math/MetricFactory.hpp>
#include <algorithm>
#include <rw/math/Math.hpp>
#include <time.h> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/chrono.hpp>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <iomanip>



Application::Application(ros::NodeHandle h):

	_n(h)

{

	initWorkCell();
	path = std::list<rw::math::Q>(0);
	running_path = std::list<rw::math::Q>(0);
	test = std::vector<rw::math::Transform3D<double> >(0);
	testQ = std::vector<rw::math::Q>(0);
	srand (1234);

	ROS_INFO("%s", "Test_1");
	while(test.size() < 1000)
	{
		rw::math::Vector3D<double> newPos((rand()%1000)/1000.0 - 0.5, (rand()%1000)/1000.0 - 0.5, (rand()%2000)/1000.0 - 1);
		rw::math::Transform3D<double> newTrans(newPos);



	
		std::vector<rw::math::Q> qVec = _solver->solve(newTrans, _state);

		if(qVec.size() != 0 and qVec[0].size() >1)
		{
			//std::stringstream buffer;
			//buffer << "TestTrans:  " << newTrans << std::endl;
			//ROS_INFO("%s", buffer.str().c_str());
			test.push_back(newTrans);
			testQ.push_back(qVec[0]);

		}

	}
	ROS_INFO("%s", "Test_2");
	f = std::ofstream( ros::package::getPath("rovi2") + "/Roadmap/" + "Data.txt");

	rovi2::Plan srv;
	srv.request.goal = toRos(testQ.at(index));
	srv.request.init = toRos(_device->getQ(_state));
	index++;
	begin = std::chrono::steady_clock::now();
	path_call.call(srv);

	
	
	

};


Application::~Application()
{
	f.close();	
	delete _workcell;
  	delete _device;
	delete _metric;
};

/************************************************************************
 * Q  -> lend from Caros
 ************************************************************************/
rw::math::Q Application::toRw(const rovi2::Q& q)
{
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

rovi2::Q Application::toRos(const rw::math::Q& q)
{
  rovi2::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}


void Application::initWorkCell()
{
	std::string path = ros::package::getPath("rovi2") + "/WorkStation_astar/WC3_Scene.wc.xml";
	_workcell = rw::loaders::WorkCellLoader::Factory::load(path);

	_device = _workcell->findDevice("UR1");

	_state =  _workcell->getDefaultState();


// Create metric weight, as motion weight in cartesian space.
	_metricWeights = rw::pathplanning::PlannerUtil::estimateMotionWeights(*_device, _device->getEnd(),_state,rw::pathplanning::PlannerUtil::WORSTCASE,1000);

	std::cout << _metricWeights << std::endl;

	// Create the metric
	//_metric = rw::math::MetricFactory::makeWeightedEuclidean<rw::math::Q>(_metricWeights);
	_metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();

	_BallErrorFrame = _workcell->findFrame("WSG50.BallError");
	_baseFrame = _workcell->findFrame("UR1.Base");


	_solver = new rw::invkin::JacobianIKSolver(_device, _BallErrorFrame, _state);

	_solver->setClampToBounds(true);


	//robot_controller = _n.subscribe("/rovi2/robot_node/Robot_state", 1, &Application::RobotCallback, this);
	//goal_controller = _n.subscribe("/ball_locator_3d/kalman_prediction", 1, &Application::GoalCallback, this);
	path_controller = _n.subscribe("/rovi2/Roadmap/Path" , 1, &Application::PathCallback, this);

	path_call = _n.serviceClient<rovi2::Plan>("/rovi2/Roadmap/StartPlan");
	//robot_call = _n.serviceClient<rovi2::MovePtp>("rovi2/robot_node/Move_nonlinear_ptp");

	moving_to = _device->getQ(_state);
	//moving_to = rw::math::Q(6,0.0,0.0,0.0,0.0,0.0,0.0);

};



void Application::RobotCallback(const rovi2::State &State)
{

	static int count = 0;
	/*if(!plan_incoming && !path.empty() && running_path.empty())
	{
		// TODO lazy collision detection
		running_path.push_back(path.front());
		path.pop_front();

	}
	_device->setQ(toRw(State.q), _state);

	if(!State.is_moving)
	{
		count++;
		if(count >= 20)
		{
			starting = true;
			count = 0;

		}

	}
	else
		count = 0;*/

	if(!path.empty())//!running_path.empty() && !State.e)
	{

		//double dist = _metric->distance(_device->getQ(_state), running_path.front());
		//std::stringstream diss;
		//diss << "Distance to next node:  " << dist << std::endl;
		//ROS_INFO("%s", diss.str().c_str());
		
		if(true)//dist < 0.05 || starting)
		{
			starting = false;
			rovi2::MovePtp srv;
			srv.request.target = toRos(path.front());//running_path.front());
			std::stringstream buffer;
			//buffer << "Robot moving to:  " << running_path.front() << std::endl;
			ROS_INFO("%s", buffer.str().c_str());
			//moving_to = running_path.front();
			path.pop_front();			
			//running_path.pop_front();

			robot_call.call(srv);

		}



	}


	


}



void Application::GoalCallback(const rovi2::position3D &position)
{




}

void Application::PathCallback(const rovi2::path &path_in)
{
	
	path.clear();
	for(int i = 0; i< path_in.data.size(); i++)
	{

		path.push_back(toRw(path_in.data.at(i)));

	}

	end = std::chrono::steady_clock::now();
	if ( !f.fail())
	{

		if(path.size() > 0)
		{
			_device->setQ(path.back(), _state);
		}
		
		rw::math::Transform3D<double> goal = _baseFrame->fTf(_BallErrorFrame, _state);
		rw::math::Vector3D<double> error = test.at(index-1).P() - goal.P();
		f << error.norm2() << '\t';
		f << (std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) << '\t';
		f << path.size() << std::endl;


	}




	if(index < 1000)
	{
			rovi2::Plan srv;
			srv.request.goal = toRos(testQ.at(index));
			_state =  _workcell->getDefaultState();
			srv.request.init = toRos(_device->getQ(_state));
			index++;
			begin = std::chrono::steady_clock::now();
			ROS_INFO("%s%i", "Index is: ", index);
			path_call.call(srv);
				
				
				
	}
	else if(index == 1000)
		ROS_INFO("Done");
	


}



int main(int argc, char **argv)
{

  ros::init(argc,argv,"Application_node");
  ros::NodeHandle n;
  
   Application Application_node(n);


  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
