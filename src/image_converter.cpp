// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include "ColorDetector.hpp"
#include "rovi2/position2D.h"

#define SHOW_INPUT_STREAM false

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  //Ball detection
  ros::Publisher position_pub;
  ColorDetector * detector;

  // Image storing
  int image_number = 0;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    std::string node_name;
    node_name = ros::this_node::getName();
    std::string param_name;
    param_name = node_name + "/camera_sub";

    std::string parameter;
    if (nh_.getParam(param_name, parameter))
    {
      ROS_INFO("Got param: %s", parameter.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'");
    }
    std::string input_topic_cam = parameter;
    std::string output_topic_cam = node_name+"/bb_cam_opencv";
    image_sub_ = it_.subscribe(input_topic_cam, 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(output_topic_cam, 1);

    // Ball detection
    std::string output_topic_position = node_name + "/pos";
    position_pub = nh_.advertise<rovi2::position2D>(output_topic_position,1000);
    detector = new ColorDetector();

    if (SHOW_INPUT_STREAM) {
        cv::namedWindow(OPENCV_WINDOW);
    }
  }

  ~ImageConverter()
  {
    if (SHOW_INPUT_STREAM) {
        cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    if (SHOW_INPUT_STREAM) {
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    }
    cv::waitKey(3);
    std::string image_name = "home/mathias/Desktop/images/calib" + std::to_string(image_number) + ".jpg";
    ROS_INFO("%s",image_name);
    cv::imwrite("/home/mathias/Desktop/images/calib0.png",cv_ptr->image);
    image_number++;

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

    // Find the ball
    std::vector<Point2f> position;
    position = detector->FindMarker(cv_ptr->image);
    ROS_INFO("Found [%ld] ball", (long int)position.size());
    if(position.size() > 0)
    {
      rovi2::position2D msg;
      msg.x = (float)position.at(0).x;
      msg.y = (float)position.at(0).y;
      position_pub.publish(msg); // Publish it
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
