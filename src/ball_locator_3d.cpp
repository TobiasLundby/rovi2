/* INCLUDES */
// ROS
#include "ros/ros.h"
#include "rovi2/position2D.h"
#include "rovi2/position3D.h"
#include "rovi2/velocityXYZ.h"

// Image
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OPENCV
//#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/video/tracking.hpp"
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
#define OUTPUT_TRIANGULATED_POINT false
#define OUTPUT_RECT_AND_UNDIST_POINT false
#define CAM_FREQ 10

#define kalman_order2 true /*True = order 2 and false = order 1*/

rovi2::position2D msg_undist_left;
rovi2::position2D msg_undist_right;
ros::Publisher position_pub_left;
ros::Publisher position_pub_right;

// NOTE: GLOBAL VALUES; EASIER FOR FINDING THE VELOCITY SINCE YOU CAN JUST SUBSCRIBE INSTEAD OF LOOKING IN THE FILE
rovi2::velocityXYZ msg_velocity;
ros::Publisher position_velocity;
int image_number = 0;

cv::Mat propagate_error_2D_to_3D(cv::Mat E_m, cv::Point2f point_2D_left, cv::Point2f point_2D_right)
{
  /* How to index using Rect_
  cv::Rect_<double>(start col, start row,width,height));
  */
  bool debug_propagate = false;

  /* Left camera */
  cv::Mat proj_l_ros = cv::Mat::zeros(3,4,CV_64F);
  proj_l_ros.at<double>(0,0) = 1333.329856;
  proj_l_ros.at<double>(0,2) = 537.019508;
  proj_l_ros.at<double>(1,1) =  1333.329856;
  proj_l_ros.at<double>(1,2) = 375.7627379;
  proj_l_ros.at<double>(2,2) = 1.0;

  /* Right camera */
  cv::Mat proj_r_ros = cv::Mat::zeros(3,4,CV_64F);
  proj_r_ros.at<double>(0,0) = 1333.329856;
  proj_r_ros.at<double>(0,2) = 537.019508;
  proj_r_ros.at<double>(0,3) = -159.033917;
  proj_r_ros.at<double>(1,1) =  1333.329856;
  proj_r_ros.at<double>(1,2) = 375.762737;
  proj_r_ros.at<double>(2,2) = 1.0;

  /* Subparts of the left projection matrix */
  cv::Mat Q_l_1 = cv::Mat::zeros(1,3,CV_64F);
  cv::Mat Q_l_2 = cv::Mat::zeros(1,3,CV_64F);
  cv::Mat Q_l_3 = cv::Mat::zeros(1,3,CV_64F);

  proj_l_ros(cv::Rect_<double>(0,0,3,1)).copyTo(Q_l_1);
  //cv::transpose(Q_l_1,Q_l_1);
  proj_l_ros(cv::Rect_<double>(0,1,3,1)).copyTo(Q_l_2);
  //cv::transpose(Q_l_2,Q_l_2);
  proj_l_ros(cv::Rect_<double>(0,2,3,1)).copyTo(Q_l_3);
  //cv::transpose(Q_l_3,Q_l_3);

  double q_l_1_4 = proj_l_ros.at<double>(0,3);
  double q_l_2_4 = proj_l_ros.at<double>(1,3);
  double q_l_3_4 = proj_l_ros.at<double>(2,3);

  /* Subparts of the right projection matrix */
  cv::Mat Q_r_1 = cv::Mat::zeros(1,3,CV_64F);
  cv::Mat Q_r_2 = cv::Mat::zeros(1,3,CV_64F);
  cv::Mat Q_r_3 = cv::Mat::zeros(1,3,CV_64F);

  proj_r_ros(cv::Rect_<double>(0,0,3,1)).copyTo(Q_r_1);
  //cv::transpose(Q_r_1,Q_r_1);
  proj_r_ros(cv::Rect_<double>(0,1,3,1)).copyTo(Q_r_2);
  //cv::transpose(Q_r_2,Q_r_2);
  proj_r_ros(cv::Rect_<double>(0,2,3,1)).copyTo(Q_r_3);
  //cv::transpose(Q_r_3,Q_r_3);

  double q_r_1_4 = proj_r_ros.at<double>(0,3);
  double q_r_2_4 = proj_r_ros.at<double>(1,3);
  double q_r_3_4 = proj_r_ros.at<double>(2,3);

  /************** INPUT PARAMETERS ************************************/
  /* Diagonals matrices of 2D variance-covariance matrices */
  // Diagonal gives only variance
  //cv::Mat E_m = cv::Mat::zeros(4,4,CV_64F);
  // E_m.at<double>(0,0) = 1.0e-10;
  // E_m.at<double>(1,1) = 1.0e-10;
  // E_m.at<double>(2,2) = 1.0e-10;
  // E_m.at<double>(3,3) = 1.0e-10;

  /* 2D points */
  //double u_l = 1;
  //double v_l = 1;
  //double u_r = 1;
  //double v_r = 1;
  double u_l = (double)point_2D_left.x;
  double v_l = (double)point_2D_left.y;
  double u_r = (double)point_2D_right.x;
  double v_r = (double)point_2D_right.y;




  /*******************************************************************/

  int num_2D_points = 2;


  /* Left partial derivatives */
  cv::Mat dA_l_du_l = cv::Mat::zeros(4,3,CV_64F);
  cv::Mat dA_l_dv_l = cv::Mat::zeros(4,3,CV_64F);
  cv::Mat db_l_du_l = cv::Mat::zeros(4,1,CV_64F);
  cv::Mat db_l_dv_l = cv::Mat::zeros(4,1,CV_64F);

  Q_l_3 = - Q_l_3;
  Q_l_3.copyTo(dA_l_du_l(cv::Rect_<double>(0,0,3,1)));
  Q_l_3.copyTo(dA_l_dv_l(cv::Rect_<double>(0,3,3,1)));
  Q_l_3 = - Q_l_3;
  db_l_du_l.at<double>(0,0) = q_l_3_4;
  db_l_dv_l.at<double>(0,3) = q_l_3_4;

  /* Right partial derivatives */
  cv::Mat dA_r_du_r = cv::Mat::zeros(4,3,CV_64F);
  cv::Mat dA_r_dv_r = cv::Mat::zeros(4,3,CV_64F);
  cv::Mat db_r_du_r = cv::Mat::zeros(4,1,CV_64F);
  cv::Mat db_r_dv_r = cv::Mat::zeros(4,1,CV_64F);

  Q_r_3 = - Q_r_3;
  Q_r_3.copyTo(dA_r_du_r(cv::Rect_<double>(0,0,3,1)));
  Q_r_3.copyTo(dA_r_dv_r(cv::Rect_<double>(0,3,3,1)));
  Q_r_3 = - Q_r_3;
  db_r_du_r.at<double>(0,0) = q_r_3_4;
  db_r_dv_r.at<double>(0,3) = q_r_3_4;

  /* A and b matrices */
  cv::Mat temp = cv::Mat::zeros(3,1,CV_64F);
  cv::Mat A = cv::Mat::zeros(2*2,3,CV_64F);
  temp = Q_l_1 - u_l*Q_l_3;
  temp.copyTo(A(cv::Rect_<double>(0,0,3,1)));
  temp = Q_l_1 - v_l*Q_l_3;
  temp.copyTo(A(cv::Rect_<double>(0,1,3,1)));
  temp = Q_r_1 - u_r*Q_r_3;
  temp.copyTo(A(cv::Rect_<double>(0,2,3,1)));
  temp = Q_r_1 - v_r*Q_r_3;
  temp.copyTo(A(cv::Rect_<double>(0,3,3,1)));

  cv::Mat b = cv::Mat::zeros(2*2,1,CV_64F);
  b.at<double>(0,0) = u_l*q_l_3_4 - q_l_1_4;
  b.at<double>(0,1) = v_l*q_l_3_4 - q_l_2_4;
  b.at<double>(0,2) = u_r*q_r_3_4 - q_r_1_4;
  b.at<double>(0,3) = v_r*q_r_3_4 - q_r_2_4;

  /* dM_left */
  cv::Mat ATA_inv = (A.t()*A).inv(cv::DECOMP_SVD);
  // du
  cv::Mat ATA_inv_du_l = - ATA_inv * (dA_l_du_l.inv(cv::DECOMP_SVD)*A+A.t()*dA_l_du_l)*ATA_inv;
  cv::Mat dM_l_du_l = ATA_inv_du_l*A.t()*b + ATA_inv*dA_l_du_l.t()*b + ATA_inv*A.t()*db_l_du_l;
  // dv
  cv::Mat ATA_inv_dv_l = - ATA_inv * (dA_l_dv_l.inv(cv::DECOMP_SVD)*A*A.t()*dA_l_dv_l)*ATA_inv;
  cv::Mat dM_l_dv_l = ATA_inv_dv_l*A.t()*b + ATA_inv*dA_l_dv_l.t()*b + ATA_inv*A.t()*db_l_dv_l;
  if(debug_propagate) std::cout << "dM_l_du_l:" << std::endl << dM_l_du_l << std::endl;
  if(debug_propagate) std::cout << "dM_l_dv_l:" << std::endl << dM_l_dv_l << std::endl;
  // Assembly to dM
  cv::Mat dM_dm_l = cv::Mat::zeros(3,2,CV_64F);
  dM_l_du_l.copyTo(dM_dm_l(cv::Rect_<double>(0,0,1,3)));
  dM_l_dv_l.copyTo(dM_dm_l(cv::Rect_<double>(1,0,1,3)));

  /* dM_right */
  //du
  cv::Mat ATA_inv_du_r = - ATA_inv * (dA_r_du_r.inv(cv::DECOMP_SVD)*A+A.t()*dA_r_du_r)*ATA_inv;
  cv::Mat dM_r_du_r = ATA_inv_du_r*A.t()*b + ATA_inv*dA_r_du_r.t()*b + ATA_inv*A.t()*db_r_du_r;
  // dv
  cv::Mat ATA_inv_dv_r = - ATA_inv * (dA_r_dv_r.inv(cv::DECOMP_SVD)*A*A.t()*dA_r_dv_r)*ATA_inv;
  cv::Mat dM_r_dv_r = ATA_inv_dv_r*A.t()*b + ATA_inv*dA_r_dv_r.t()*b + ATA_inv*A.t()*db_r_dv_r;
  // Assembly to dM
  cv::Mat dM_dm_r = cv::Mat::zeros(3,2,CV_64F);
  dM_r_du_r.copyTo(dM_dm_r(cv::Rect_<double>(0,0,1,3)));
  dM_r_dv_r.copyTo(dM_dm_r(cv::Rect_<double>(1,0,1,3)));

  /* dM */
  cv::Mat JM = cv::Mat::zeros(3,4,CV_64F);
  dM_dm_l.copyTo(JM(cv::Rect_<double>(0,0,2,3)));
  dM_dm_r.copyTo(JM(cv::Rect_<double>(2,0,2,3)));

  if(debug_propagate) std::cout << "JM:" << std::endl << JM << std::endl;
  if(debug_propagate) std::cout << "JM.t:" << std::endl << JM.t() << std::endl;

  cv::Mat E_M = cv::Mat::zeros(3,3,CV_64F);
  if(debug_propagate) std::cout << "E_M:" << std::endl << E_M << std::endl;
  E_M = JM*E_m*JM.t();

  return E_M;
}

std::vector<float> calculate_3D_point(double left_x, double left_y, double right_x, double right_y)
{
    Mat left_point(1, 1, CV_64FC2);
    Mat right_point(1, 1, CV_64FC2);
    Mat point_3D(1, 1, CV_64FC4);
    left_point.at<Vec2d>(0)[0] = left_x;
    left_point.at<Vec2d>(0)[1] = left_y;
    right_point.at<Vec2d>(0)[0] = right_x;
    right_point.at<Vec2d>(0)[1] = right_y;

    /* LEFT CALIBRATION */
    // double img_width_l = 1024;
    // double img_height_l = 768;

    Mat camera_matrix_l = Mat::zeros(3,3,CV_64F);
    camera_matrix_l.at<double>(0,0) = 1366.940645;
    // 0,1: zero
    camera_matrix_l.at<double>(0,2) = 510.148527;
    // 1,0: zero
    camera_matrix_l.at<double>(1,1) = 1367.181188;
    camera_matrix_l.at<double>(1,2) = 364.424779;
    // 2,0: zero
    // 2,1: zero
    camera_matrix_l.at<double>(2,2) = 1;

    Mat dist_coeffs_l = Mat::zeros(1,5,CV_64F);
    dist_coeffs_l.at<double>(0,0) = -0.468731;
    dist_coeffs_l.at<double>(0,1) =  0.476022;
    dist_coeffs_l.at<double>(0,2) =  0.004279;
    dist_coeffs_l.at<double>(0,3) =  0.000135;
    dist_coeffs_l.at<double>(0,4) = -0.633893;

    Mat rectification_l = Mat::zeros(3,3,CV_64F);
    // setIdentity(rectification_l);
    rectification_l.at<double>(0,0) =  0.999871;
    rectification_l.at<double>(0,1) =  0.004958;
    rectification_l.at<double>(0,2) = -0.015269;
    rectification_l.at<double>(1,0) = -0.005083;
    rectification_l.at<double>(1,1) =  0.999954;
    rectification_l.at<double>(1,2) = -0.008142;
    rectification_l.at<double>(2,0) =  0.015228;
    rectification_l.at<double>(2,1) =  0.008219;
    rectification_l.at<double>(2,2) =  0.999850;

    Mat proj_l_ros = Mat::zeros(3,4,CV_64F);
    proj_l_ros.at<double>(0,0) = 1333.329856;
    proj_l_ros.at<double>(0,2) = 537.019508;
    proj_l_ros.at<double>(1,1) =  1333.329856;
    proj_l_ros.at<double>(1,2) = 375.7627379;
    proj_l_ros.at<double>(2,2) = 1.0;

    /* RIGHT CALIBRATION */
    // double img_width_r = 1024;
    // double img_height_r = 768;

    Mat camera_matrix_r = Mat::zeros(3,3,CV_64F);
    camera_matrix_r.at<double>(0,0) = 1372.805317;
    // 0,1: zero
    camera_matrix_r.at<double>(0,2) = 503.266067;
    // 1,0: zero
    camera_matrix_r.at<double>(1,1) = 1373.724158;
    camera_matrix_r.at<double>(1,2) = 383.804262;
    // 2,0: zero
    // 2,1: zero
    camera_matrix_r.at<double>(2,2) = 1;

    Mat dist_coeffs_r = Mat::zeros(1,5,CV_64F);
    dist_coeffs_r.at<double>(0,0) = -0.468950;
    dist_coeffs_r.at<double>(0,1) =  0.542028;
    dist_coeffs_r.at<double>(0,2) =  0.002906;
    dist_coeffs_r.at<double>(0,3) =  0.003794;
    dist_coeffs_r.at<double>(0,4) = -0.828583;

    Mat rectification_r = Mat::zeros(3,3,CV_64F);
    // setIdentity(rectification_r);
    rectification_r.at<double>(0,0) =  0.999671;
    rectification_r.at<double>(0,1) =  0.004195;
    rectification_r.at<double>(0,2) = -0.025319;
    rectification_r.at<double>(1,0) = -0.003988;
    rectification_r.at<double>(1,1) =  0.999958;
    rectification_r.at<double>(1,2) =  0.008232;
    rectification_r.at<double>(2,0) =  0.025352;
    rectification_r.at<double>(2,1) = -0.008128;
    rectification_r.at<double>(2,2) =  0.999646;

    Mat proj_r_ros = Mat::zeros(3,4,CV_64F);
    proj_r_ros.at<double>(0,0) = 1333.329856;
    proj_r_ros.at<double>(0,2) = 537.019508;
    proj_r_ros.at<double>(0,3) = -159.033917;
    proj_r_ros.at<double>(1,1) =  1333.329856;
    proj_r_ros.at<double>(1,2) = 375.762737;
    proj_r_ros.at<double>(2,2) = 1.0;

    Mat left_point_undistorted(1, 1, CV_64FC2);
    Mat right_point_undistorted(1, 1, CV_64FC2);

    undistortPoints(left_point, left_point_undistorted, camera_matrix_l, dist_coeffs_l, rectification_l, camera_matrix_l);
    undistortPoints(right_point, right_point_undistorted, camera_matrix_r, dist_coeffs_r, rectification_r, camera_matrix_r);
    //ROS_ERROR("Left: %f, %f", (float)left_point_undistorted.at<Vec2d>(0)[0], (float)left_point_undistorted.at<Vec2d>(0)[1]);
    //ROS_ERROR("Right: %f, %f", (float)right_point_undistorted.at<Vec2d>(0)[0], (float)right_point_undistorted.at<Vec2d>(0)[1]);


    cv::Mat E_m = cv::Mat::zeros(4,4,CV_64F);
    E_m.at<double>(0,0) = 1.0e-0;
    E_m.at<double>(1,1) = 1.0e-0;
    E_m.at<double>(2,2) = 1.0e-0;
    E_m.at<double>(3,3) = 1.0e-0;

    cv::Point2f point_2D_left;
    cv::Point2f point_2D_right;
    point_2D_left.x = (float)left_point_undistorted.at<Vec2d>(0)[0];
    point_2D_left.y = (float)left_point_undistorted.at<Vec2d>(0)[1];
    point_2D_right.x = (float)right_point_undistorted.at<Vec2d>(0)[0];
    point_2D_right.y = (float)right_point_undistorted.at<Vec2d>(0)[1];

    std::stringstream buffer_left;
    buffer_left <<  "Left" << point_2D_left.x <<  ", " << point_2D_left.y << std::endl << "Right: " << point_2D_right.x <<  ", " << point_2D_right.y << std::endl;
    ROS_ERROR("%s", buffer_left.str().c_str());

    cv::Mat E_M = propagate_error_2D_to_3D(E_m, point_2D_left, point_2D_right);
    buffer_left <<  "Variance: x=" <<  E_M.at<float>(0,0) <<  " y=" << E_M.at<float>(1,1) << " z: " << E_M.at<float>(2,2) << std::endl;
    ROS_ERROR("%s", buffer_left.str().c_str());
    //ROS_ERROR("Variance: x=%f, \ty=%f, \tz=%f", E_M.at<float>(0,0), E_M.at<float>(1,1), E_M.at<float>(2,2));



    msg_undist_left.x = (float)left_point_undistorted.at<Vec2d>(0)[0];
    msg_undist_left.y = (float)left_point_undistorted.at<Vec2d>(0)[1];
    position_pub_left.publish(msg_undist_left); // Publish it
    msg_undist_right.x = (float)right_point_undistorted.at<Vec2d>(0)[0];
    msg_undist_right.y = (float)right_point_undistorted.at<Vec2d>(0)[1];
    position_pub_right.publish(msg_undist_right); // Publish it

    if (OUTPUT_RECT_AND_UNDIST_POINT) {
        std::stringstream buffer_left;
        buffer_left <<  "Left" << std::endl << "Org point: " << left_point.at<Vec2d>(0)[0] <<  ", " << left_point.at<Vec2d>(0)[1] << std::endl << "Undistorted point: " << left_point_undistorted.at<Vec2d>(0)[0] <<  ", " << left_point_undistorted.at<Vec2d>(0)[1] << std::endl;
        ROS_ERROR("%s", buffer_left.str().c_str());

        std::stringstream buffer_right;
        buffer_right <<  "Right" << std::endl << "Org point: " << right_point.at<Vec2d>(0)[0] <<  ", " << right_point.at<Vec2d>(0)[1] << std::endl << "Undistorted point: " << right_point_undistorted.at<Vec2d>(0)[0] <<  ", " << right_point_undistorted.at<Vec2d>(0)[1] << std::endl;
        ROS_ERROR("%s", buffer_right.str().c_str());
    }

    triangulatePoints(proj_l_ros,proj_r_ros,left_point_undistorted,right_point_undistorted,point_3D);

    std::vector<float> result_vec = {(float)(point_3D.at<Vec2d>(0)[0]/point_3D.at<Vec2d>(0)[3]), // THE RESULT IS LENGTH 4!!!!!!
                                   (float)(point_3D.at<Vec2d>(0)[1]/point_3D.at<Vec2d>(0)[3]),
                                   (float)(point_3D.at<Vec2d>(0)[2]/point_3D.at<Vec2d>(0)[3])};

    if (OUTPUT_TRIANGULATED_POINT) {
        std::stringstream buffer;
        buffer << std::endl << "3D point:" << std::endl << result_vec.at(0) << std::endl << result_vec.at(1) << std::endl << result_vec.at(2) << std::endl;
        ROS_ERROR("%s", buffer.str().c_str());
    }
    return result_vec;
}

void callback(
    const sensor_msgs::ImageConstPtr& image_left,
    const sensor_msgs::ImageConstPtr& image_right,
    /*image_transport::Publisher& image_pub_left_ptr,
    image_transport::Publisher& image_pub_right_ptr,*/
    /*ros::Publisher& pos_pub_left_ptr,
    ros::Publisher& pos_pub_right_ptr,*/
    ros::Publisher& pos_pub_triangulated_ptr,
    ColorDetector& deterctor_left_obj_ptr,
    ColorDetector& deterctor_right_obj_ptr,
    cv::KalmanFilter KF_ptr,
    ros::Publisher& kalman_pub_estimate_ptr,
    ros::Publisher& kalman_pub_prediction_ptr
)
{
    /* PUBLISH of diables arguments are commented out at publish point below*/
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
    cv::waitKey(3); // Waitkey is nessecary for proper GUI showing

    // Output modified video stream - disabled due to limited arguments and not needed elesewhere
    //image_pub_left_ptr.publish(cv_left_ptr->toImageMsg());
    //image_pub_right_ptr.publish(cv_right_ptr->toImageMsg());

    // NOTE: Find the left ball
    std::vector<Point2f> position_left;
    position_left = deterctor_left_obj_ptr.FindMarker(cv_left_ptr->image);
    ROS_INFO("Found [%ld] ball", (long int)position_left.size());
    // Publish the left ball location - disabled due to limited arguments and not needed elesewhere
    if(position_left.size() > 0)
    {
        rovi2::position2D msg;
        msg.x = (float)position_left.at(0).x;
        msg.y = (float)position_left.at(0).y;
        //pos_pub_left_ptr.publish(msg); // Publish it
    }

    // NOTE: Find the right ball
    std::vector<Point2f> position_right;
    position_right = deterctor_right_obj_ptr.FindMarker(cv_right_ptr->image);
    ROS_INFO("Found [%ld] ball", (long int)position_right.size());
    // Publish the right ball location - disabled due to limited arguments and not needed elesewhere
    if(position_right.size() > 0)
    {
        rovi2::position2D msg;
        msg.x = (float)position_right.at(0).x;
        msg.y = (float)position_right.at(0).y;
        //pos_pub_right_ptr.publish(msg); // Publish it
    }

    // Prepare variables
    rovi2::position3D msg;
    Mat_<float> measurement(3,1);
    measurement.setTo(Scalar(0));
    // NOTE: Triangulate if a ball has been found
    if(position_right.size() > 0 and position_left.size() > 0 ) {
        // Prepare vector for traingulation and triangulate the 2 2D points
        std::vector<float> triangluated_point;
        triangluated_point = calculate_3D_point((double)position_left.at(0).x, (double)position_left.at(0).y, (double)position_right.at(0).x, (double)position_right.at(0).y);

        if(triangluated_point.size() > 0)
        {
            // Publish the triangualted point before Kalman
            msg.x = (float)triangluated_point.at(0);
            msg.y = (float)triangluated_point.at(1);
            msg.z = (float)triangluated_point.at(2);
            pos_pub_triangulated_ptr.publish(msg); // Publish it

            // Note triangulated point in the right format for correcting
            measurement(0) = (float)triangluated_point.at(0);
            measurement(1) = (float)triangluated_point.at(1);
            measurement(2) = (float)triangluated_point.at(2);

            // Correct the estimate and publish the corrected estimate
            Mat estimated = KF_ptr.correct(measurement);
            msg.x = estimated.at<float>(0);
            msg.y = estimated.at<float>(1);
            msg.z = estimated.at<float>(2);
            kalman_pub_estimate_ptr.publish(msg); // Publish it

            // Velocity publisher and calculator
            // NOTE: THIS IS MADE WITH A GLOBAL VARIABLE - DELETE LATER
            msg_velocity.x_dot = estimated.at<float>(3);
            msg_velocity.y_dot = estimated.at<float>(4);
            msg_velocity.z_dot = estimated.at<float>(5);
            msg_velocity.p_dot = sqrt(pow(estimated.at<float>(3),2)+pow(estimated.at<float>(4),2)+pow(estimated.at<float>(5),2));
            position_velocity.publish(msg_velocity); // Publish it

            // ROS_ERROR("Pos:\t%f \t%f \t%f \n", estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2));
            // ROS_ERROR("Vel:\t%f \t%f \t%f \n", estimated.at<float>(3), estimated.at<float>(4), estimated.at<float>(5));
            // if (kalman_order2)
            //     ROS_ERROR("Acc:\t%f \t%f \t%f \n\n", estimated.at<float>(6), estimated.at<float>(7), estimated.at<float>(8));

            // Save image
            if(false)
            {
                // Save to file
                //std::string calib_path = "/home/mathias/Desktop/image_log"; // This is also used for saving the images
                std::string calib_path = "/home/tobiaslundby/Desktop/image_log"; // This is also used for saving the images
                std::ofstream file;
                file.open(calib_path+"/velocities.log",std::ios::app);
                file << image_number << "\t";
                file << (float)triangluated_point.at(0) << "\t";
                file << (float)triangluated_point.at(1) << "\t";
                file << (float)triangluated_point.at(2) << "\t";
                file << estimated.at<float>(0) << "\t";
                file << estimated.at<float>(1) << "\t";
                file << estimated.at<float>(2) << "\t";
                file << msg_velocity.x_dot << "\t";
                file << msg_velocity.y_dot << "\t";
                file << msg_velocity.z_dot << "\t";
                file << msg_velocity.p_dot << "\t";
                file << std::endl;
                file.close();

                // Save image

                std::ostringstream name_left;
                name_left << calib_path << "/image_left/" << std::to_string(image_number) << ".jpg";
                std::ostringstream name_right;
                name_right << calib_path << "/image_right/" << std::to_string(image_number) << ".jpg";
                cv::imwrite(name_left.str(),cv_left_ptr->image);
                cv::imwrite(name_right.str(),cv_right_ptr->image);
            }
            image_number++;
        }
    }
    // Predict and publish the prediction even if no ball has been deteced
    Mat prediction = KF_ptr.predict();
    msg.x = prediction.at<float>(0);
    msg.y = prediction.at<float>(1);
    msg.z = prediction.at<float>(2);
    kalman_pub_prediction_ptr.publish(msg); // Publish it
}

int main(int argc, char** argv)
{
  // cv::Mat E_m = cv::Mat::zeros(4,4,CV_64F);
  // E_m.at<double>(0,0) = 1.0e-0;
  // E_m.at<double>(1,1) = 1.0e-0;
  // E_m.at<double>(2,2) = 1.0e-0;
  // E_m.at<double>(3,3) = 1.0e-0;
  //
  // cv::Point2f point_2D_left;
  // cv::Point2f point_2D_right;
  // point_2D_left.x = 1.0;
  // point_2D_left.y = 1.0;
  // point_2D_right.x = 1.0;
  // point_2D_right.y = 1.0;
  //
  // cv::Mat E_M = propagate_error_2D_to_3D(E_m, point_2D_left, point_2D_right);
  // std::stringstream buf;
  // buf << E_M.at<double>(0,0) << "\t" << E_M.at<double>(0,1) << "\t" << E_M.at<double>(0,2) << std::endl;
  // buf << E_M.at<double>(1,0) << "\t" << E_M.at<double>(1,1) << "\t" << E_M.at<double>(1,2) << std::endl;
  // buf << E_M.at<double>(2,0) << "\t" << E_M.at<double>(2,1) << "\t" << E_M.at<double>(2,2) << std::endl;
  //
  // ROS_ERROR("E_M \n %s",buf.str().c_str());

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

    // NOTE: Subcribe to input camera feeds
    std::string input_topic_cam_left = parameter_left;
    std::string input_topic_cam_right = parameter_right;
    ROS_INFO("Left  image topic: %s", input_topic_cam_left.c_str());
    ROS_INFO("Right image topic: %s", input_topic_cam_right.c_str());
    message_filters::Subscriber<sensor_msgs::Image> image_left(nh_, input_topic_cam_left, 1);
    message_filters::Subscriber<sensor_msgs::Image> image_right(nh_, input_topic_cam_right, 1);

    // NOTE: Setup the synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTiros::Publisher position_pub_left;me takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_left, image_right);

    // NOTE: Image conversion - NOT USED ANYMORE AS TOPIC BUT AS INTERNAL VALUE
    // std::string output_topic_cam_left = node_name + "/bb_cam_opencv_left";
    // std::string output_topic_cam_right = node_name + "/bb_cam_opencv_right";
    // ROS_INFO("Left  OpenCV image topic (output): %s", output_topic_cam_left.c_str());
    // ROS_INFO("Right OpenCV image topic (output): %s", output_topic_cam_right.c_str());
    // image_transport::ImageTransport it_(nh_);
    // image_transport::Publisher image_pub_left = it_.advertise(output_topic_cam_left, 1);
    // image_transport::Publisher image_pub_right = it_.advertise(output_topic_cam_right, 1);

    // NOTE: Position publisher - LEFT AND RIGHT POS NOT USED ANYMORE AS TOPIC BUT AS INTERNAL VALUE
    // std::string output_topic_position_left = node_name + "/pos_left";
    // std::string output_topic_position_right = node_name + "/pos_right";
    std::string output_topic_position_triangulated = node_name + "/pos_triangulated";
    // ros::Publisher position_pub_left = nh_.advertise<rovi2::position2D>(output_topic_position_left,1);
    // ros::Publisher position_pub_right = nh_.advertise<rovi2::position2D>(output_topic_position_right,1);
    ros::Publisher position_pub_triangulated = nh_.advertise<rovi2::position3D>(output_topic_position_triangulated,1);

    // NOTE: Ball detector objects; two have been made if different settings are required later
    ColorDetector detector_left;
    detector_left.set_result_window_name("Left result 2D");
    ColorDetector detector_right;
    detector_right.set_result_window_name("Right result 2D");

    // NOTE: Kalman filter
    float delta_t = 1.0/CAM_FREQ;
    KalmanFilter KF;
    if (kalman_order2) {
        KF.init(9, 3, 0);
        // intialization of KF...
        KF.transitionMatrix = (Mat_<float>(9, 9) <<
        1,0,0,delta_t,0,0,0.5*pow(delta_t,2),0,0,
        0,1,0,0,delta_t,0,0,0.5*pow(delta_t,2),0,
        0,0,1,0,0,delta_t,0,0,0.5*pow(delta_t,2),
        0,0,0,delta_t,0,0,1,0,0,
        0,0,0,0,delta_t,0,0,1,0,
        0,0,0,0,0,delta_t,0,0,1,
        0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,1);
        //Initial state - start pos estimate is (x,y,z)=(0,0,1.20)
        KF.statePre.at<float>(0) = 0; //x
        KF.statePre.at<float>(1) = 0; //y
        KF.statePre.at<float>(2) = 1.20; //z
        KF.statePre.at<float>(3) = 0; //xdot
        KF.statePre.at<float>(4) = 0; //ydot
        KF.statePre.at<float>(5) = 0; //zdot
        KF.statePre.at<float>(6) = 0; //xdotdot
        KF.statePre.at<float>(7) = 0; //ydotdot
        KF.statePre.at<float>(8) = 0; //zdotdot
        // Set the matrices to identity - DOCUMENTED ELSEWHERE WHY
        setIdentity(KF.measurementMatrix);
        /* Process or state noise - w in x_(t+1)=Ax_t + w - error/noise in the model / prediction*/
        setIdentity(KF.processNoiseCov, Scalar::all(0.02)); // accuracy of prediction
        /* Measurement noise - v in y_t=Cx_t + v - error/noise in the measurement; a measurement is only certain to a accuracy */
        setIdentity(KF.measurementNoiseCov, Scalar::all(0.05)); //measurement can deviate with XX
        /* Initial error */
        setIdentity(KF.errorCovPost, Scalar::all(1)); // we are unsure about the inital value
    } else {
        KF.init(6, 3, 0);
        // intialization of KF...
        KF.transitionMatrix = (Mat_<float>(6,6) <<
        1,0,0,delta_t,0,0,
        0,1,0,0,delta_t,0,
        0,0,1,0,0,delta_t,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1);
        //Initial state - start pos estimate is (x,y,z)=(0,0,1.20)
        KF.statePre.at<float>(0) = 0; //x
        KF.statePre.at<float>(1) = 0; //y
        KF.statePre.at<float>(2) = 1.20; //z
        KF.statePre.at<float>(3) = 0; //xdot
        KF.statePre.at<float>(4) = 0; //ydot
        KF.statePre.at<float>(5) = 0; //zdot
        // Set the matrices to identity - DOCUMENTED ELSEWHERE WHY
        setIdentity(KF.measurementMatrix);
        /* Process or state noise - w in x_(t+1)=Ax_t + w - error/noise in the model / prediction*/
        setIdentity(KF.processNoiseCov, Scalar::all(0.02)); // accuracy of prediction
        /* Measurement noise - v in y_t=Cx_t + v - error/noise in the measurement; a measurement is only certain to a accuracy */
        setIdentity(KF.measurementNoiseCov, Scalar::all(0.05)); //measurement can deviate with XX
        /* Initial error */
        setIdentity(KF.errorCovPost, Scalar::all(1)); // we are unsure about the inital value
    }


    // NOTE: Kalman publisher
    std::string kalman_estimate_str = node_name + "/kalman_estimate";
    std::string kalman_prediction_str = node_name + "/kalman_prediction";
    ros::Publisher kalman_pub_estimate = nh_.advertise<rovi2::position3D>(kalman_estimate_str,1);
    ros::Publisher kalman_pub_prediction = nh_.advertise<rovi2::position3D>(kalman_prediction_str,1);

    // NOTE: THIS IS MADE WITH A GLOBAL VARIABLE - DELETE LATER
    std::string output_topic_position_left = ros::this_node::getName() + "/pos_left";
    std::string output_topic_position_right = ros::this_node::getName() + "/pos_right";
    position_pub_left = nh_.advertise<rovi2::position2D>(output_topic_position_left,1);
    position_pub_right = nh_.advertise<rovi2::position2D>(output_topic_position_right,1);

    // NOTE: THIS IS MADE WITH A GLOBAL VARIABLE - DELETE LATER
    std::string output_topic_velocity = ros::this_node::getName() + "/velocity";
    position_velocity = nh_.advertise<rovi2::velocityXYZ>(output_topic_velocity,1);

    // NOTE: Register the callback function, be aware of the argument limit of boost (if more arguments are needed then these can be passed using a vector structure or similar with multiple elements)
    sync.registerCallback(boost::bind(&callback, _1, _2,
        /*image_pub_left, image_pub_right,*/
        /*position_pub_left, position_pub_right,*/ position_pub_triangulated,
        detector_left, detector_right,
        KF,kalman_pub_estimate,kalman_pub_prediction));

    // NOTE: Leave the rest of the scheduling to ROS
    while( ros::ok() ){
        ros::spin();
    }

    return EXIT_SUCCESS;
}
