/* INCLUDES */
// ROS
#include "ros/ros.h"
#include "rovi2/position2D.h"

// Image
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OPENCV
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>

// Subsriber syncrhonizer
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Custom classes
#include "ColorDetector.hpp"


/* NAMESPACES */

/* DEFINES */
#define SHOW_INPUT_STREAM true

void callback(
    const sensor_msgs::ImageConstPtr& image_left,
    const sensor_msgs::ImageConstPtr& image_right,
    image_transport::Publisher& image_pub_left_ptr,
    image_transport::Publisher& image_pub_right_ptr,
    ros::Publisher& pos_pub_left_ptr,
    ros::Publisher& pos_pub_right_ptr,
    ColorDetector& deterctor_left_obj_ptr,
    ColorDetector& deterctor_right_obj_ptr
)
{
  cv_bridge::CvImagePtr cv_left_ptr, cv_right_ptr;
  try
  {
    cv_left_ptr = cv_bridge::toCvCopy(image_left, sensor_msgs::image_encodings::BGR8);
    cv_right_ptr = cv_bridge::toCvCopy(image_right, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  if (SHOW_INPUT_STREAM) {
      cv::imshow("Left input", cv_left_ptr->image);
      cv::imshow("Right input", cv_right_ptr->image);
  }
  cv::waitKey(3);

  // Output modified video stream
  image_pub_left_ptr.publish(cv_left_ptr->toImageMsg());
  image_pub_right_ptr.publish(cv_right_ptr->toImageMsg());

  // Find the left ball
  std::vector<Point2f> position_left;
  position_left = deterctor_left_obj_ptr.FindMarker(cv_left_ptr->image);
  ROS_INFO("Found [%ld] ball", (long int)position_left.size());
  if(position_left.size() > 0)
  {
    rovi2::position2D msg;
    msg.x = (float)position_left.at(0).x;
    msg.y = (float)position_left.at(0).y;
    pos_pub_left_ptr.publish(msg); // Publish it
  }

  // Find the right ball
  std::vector<Point2f> position_right;
  position_right = deterctor_right_obj_ptr.FindMarker(cv_right_ptr->image);
  ROS_INFO("Found [%ld] ball", (long int)position_right.size());
  if(position_right.size() > 0)
  {
    rovi2::position2D msg;
    msg.x = (float)position_right.at(0).x;
    msg.y = (float)position_right.at(0).y;
    pos_pub_right_ptr.publish(msg); // Publish it
  }

  // Triangulation here
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_locator_3d");

    ros::NodeHandle nh_; // Create node handler

    // Subscribe to input video feed
    std::string node_name;
    node_name = ros::this_node::getName();
    std::string param_name_left, param_name_right;
    param_name_left = node_name + "/camera_sub_left";
    param_name_right = node_name + "/camera_sub_right";

    std::string parameter_left;
    if (nh_.getParam(param_name_left, parameter_left))
    {
        ROS_INFO("Got param: %s", parameter_left.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get parameters");
    }
    std::string parameter_right;
    if (nh_.getParam(param_name_right, parameter_right))
    {
        ROS_INFO("Got param: %s", parameter_right.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get parameters");
    }

    std::string input_topic_cam_left = parameter_left;
    std::string input_topic_cam_right = parameter_right;

    ROS_INFO("Left  image topic: %s", input_topic_cam_left.c_str());
    ROS_INFO("Right image topic: %s", input_topic_cam_right.c_str());

    message_filters::Subscriber<sensor_msgs::Image> image_left(nh_, input_topic_cam_left, 1);
    message_filters::Subscriber<sensor_msgs::Image> image_right(nh_, input_topic_cam_right, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTiros::Publisher position_pub_left;me takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_left, image_right);

    // Image conversion
    std::string output_topic_cam_left = node_name + "/bb_cam_opencv_left";
    std::string output_topic_cam_right = node_name + "/bb_cam_opencv_right";
    ROS_INFO("Left  OpenCV image topic (output): %s", output_topic_cam_left.c_str());
    ROS_INFO("Right OpenCV image topic (output): %s", output_topic_cam_right.c_str());
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher image_pub_left = it_.advertise(output_topic_cam_left, 1);
    image_transport::Publisher image_pub_right = it_.advertise(output_topic_cam_right, 1);

    // Position publisher
    std::string output_topic_position_left = node_name + "/pos_left";
    std::string output_topic_position_right = node_name + "/pos_right";
    ros::Publisher position_pub_left = nh_.advertise<rovi2::position2D>(output_topic_position_left,1000);
    ros::Publisher position_pub_right = nh_.advertise<rovi2::position2D>(output_topic_position_right,1000);

    // Ball detector objects; two have been made if different settings are required later
    ColorDetector detector_left;
    detector_left.set_result_window_name("Left result 2D");
    ColorDetector detector_right;
    detector_right.set_result_window_name("Right result 2D");

    sync.registerCallback(boost::bind(&callback, _1, _2, image_pub_left, image_pub_right, position_pub_left, position_pub_right, detector_left, detector_right));

    while( ros::ok() ){
        ros::spin();
    }

    return EXIT_SUCCESS;
}
