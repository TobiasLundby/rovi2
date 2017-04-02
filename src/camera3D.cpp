#include "ros/ros.h"
#include "rovi2/position2D.h"
#include "rovi2/position3D.h"


bool updated_left = false;
bool updated_right = false;
rovi2::position2D left_pos2D;
rovi2::position2D right_pos2D;

void callback_left(rovi2::position2D msg)
{
  left_pos2D = msg;
  updated_left = true;
}

void callback_left(rovi2::position2D msg)
{
  right_pos2D = msg;
  updated_right = true;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"camera3D")
  ros::NodeHandle n;

  ros::Subscriber sub_left = n.subscribe("/image_converter_left/pos",1000,callback_left);
  ros::Subscriber sub_right = n.subscribe("/image_converter_right/pos",1000,callback_right);
  ros::Publisher pub = n.advertise<rovi2::postion3D>("/camera3D/pos",1000);
  Stereopsis stereo_calc = Stereopsis("a_path")
  while(ros::ok())
  {
    if(updated_left and updated_right)
    {
      updated_left = false;
      updated_right = false;

      //calculate 3D point
      stereo_calc.set_left_point((float)left_pos2D.x,(float)left_pos2D.y);
      stereo_calc.set_right_point((float)right_pos2D.x,(float)right_pos2D.y);
      std::vector<float> point3D = stereo_calc.calculate_3D_point();
      rovi2::position3D msg;
      msg.x = point3D[0];
      msg.y = point3D[1];
      msg.z = point3D[2];
      pub.publish(msg);
    }

    ros::spinOnce();
  }
  return 0;
}
