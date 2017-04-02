#include "ColorDetector.hpp"
#include <iostream>
#define DEBUG true
ColorDetector::ColorDetector(){
  // All the parameters below are used for the blob detector - only some of them are used because motion blur introduces different shapes etc.
  // Change thresholds
  //params.minThreshold = 0;
  //params.maxThreshold = 100;
  params.blobColor = 255; // This parameter is used instead of the threadholds to detect the white color
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = CV_PI*pow(20,2); // Radius 50px
  params.maxArea = CV_PI*pow(500,2); // Radius 100px
  // Filter by Circularity - we do not do this parameter due to motion blur
  params.filterByCircularity = false;
  params.minCircularity = 0.5;
  // Filter by Convexity - we do not use this parameter to ensure detection
  params.filterByConvexity = false;
  //params.minConvexity = 0.87;
  // Filter by Inertia - not used but it means "the inertial resistance of the blob to rotation about its principal axes"
  params.filterByInertia = false;
  //params.minInertiaRatio = 0.01;
  //params.maxInertiaRatio = 0.5;
  detector = SimpleBlobDetector::create(params); // Set up detector with params
}

std::vector<Point2f> ColorDetector::FindMarker(Mat &image) {
  // Create / convert images to color spaces for processing
  Mat image_hsv, image_gray;
  cvtColor(image, image_hsv,  COLOR_BGR2HSV); // Convert from BGR to HSV
  cvtColor(image, image_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY

  // Mask red parts
  Mat mask_MR, image_masked_MR;
  inRange(image_hsv, Scalar(hsv_h_low_MR, hsv_s_low_MR, hsv_v_low_MR), Scalar(hsv_h_upper_MR, hsv_s_upper_MR, hsv_v_upper_MR), mask_MR); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask_MR, mask_MR, Mat(), Point(-1,-1)); // Enhance the areas in the image
  image_gray.copyTo(image_masked_MR, mask_MR);
  if(DEBUG)
  {
    imshow("img_hsv",image_hsv);
    imshow("mask_MR",mask_MR);
    imshow("image_mask_MR",image_masked_MR);
  }


  // Detect blobs.
  std::vector<KeyPoint> keypoints_MR;
  detector->detect( mask_MR, keypoints_MR);
  if(DEBUG)
  {
    drawKeypoints(image,keypoints_MR,image,Scalar(0,255,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("image",image);
  }

  std::vector< Point2f > output_points;
  for( int i=0; i<keypoints_MR.size(); i++)
    output_points.push_back(keypoints_MR.at(i).pt);

  return output_points;
}
