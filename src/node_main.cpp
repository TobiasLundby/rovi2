#include "RobotNode.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "RobotNode");
	ros::NodeHandle n;
	RobotNode_ros myRobot(n);
	ros::spin();

	return 0;

};
