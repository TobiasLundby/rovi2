//#include "functions.hpp"

// * Inspiration for blob detection: https://www.learnopencv.com/blob-detection-using-opencv-python-c/

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

// Namespaces
using namespace cv;
// Defines
#define HUE_ORANGE     11                      /* 0-22 */
#define HUE_YELLOW     30                      /* 22-38 */
#define HUE_GREEN      60                      /* 38-75 */
#define HUE_BLUE       100                      /* 75-130 */
#define HUE_VIOLET     145                      /* 130-160 */
#define HUE_RED        160                      /* 160-179 */
 // Custom colors
#define HUE_YELLOW_OWN 15                      /* 22-38 */
#define HUE_BLUE_OWN   120                      /* 75-130 */

class ColorDetector {

public:
  ColorDetector();
  std::vector<Point2f> FindMarker(Mat &image);
private:
  int dilate_color_iterations = 1;

  SimpleBlobDetector::Params params;
  Ptr<SimpleBlobDetector> detector;

  // *** MARKER BLUE ***
  int hsv_h_base_MB        = HUE_BLUE_OWN;
  int hsv_h_sensitivity_MB = 25; // 5 for big green led and 24 for most other
  int hsv_h_low_MB         = hsv_h_base_MB - hsv_h_sensitivity_MB; //hsv_h_base - hsv_h_sensitivity;
  int hsv_h_upper_MB       = hsv_h_base_MB + hsv_h_sensitivity_MB;//hsv_h_base + hsv_h_sensitivity;
  int hsv_s_low_MB         = 30; //100;
  int hsv_s_upper_MB       = 255;
  int hsv_v_low_MB         = 30; //100;
  int hsv_v_upper_MB       = 255;

  // *** MARKER RED *** (the red is shifted into the yellow area)
  int hsv_h_base_MR        = HUE_YELLOW_OWN;
  int hsv_h_sensitivity_MR = 15; // 5 for big green led and 24 for most other
  int hsv_h_low_MR         = 0;//hsv_h_base_MR - hsv_h_sensitivity_MR; //hsv_h_base - hsv_h_sensitivity;
  int hsv_h_upper_MR       = 30;//hsv_h_base_MR + hsv_h_sensitivity_MR;//hsv_h_base + hsv_h_sensitivity;
  int hsv_s_low_MR         = 160; //100;
  int hsv_s_upper_MR       = 255;
  int hsv_v_low_MR         = 160; //100;
  int hsv_v_upper_MR       = 255;
};
