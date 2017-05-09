//#include "functions.hpp"

// * Inspiration for blob detection: https://www.learnopencv.com/blob-detection-using-opencv-python-c/

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <string>

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
  void set_result_window_name(std::string name);
  std::vector<double> estimate_variance(Mat &image);
private:
    std::string window_name = "Result 2D";

    int erode_color_iterations  = 1;
    int dilate_color_iterations = 3;

    SimpleBlobDetector::Params params;
    Ptr<SimpleBlobDetector> detector;

    // *** MARKER RED / RED BALL *** (the red is shifted into the yellow area)
    int hsv_h_base_MR        = 55; // 55 is almost green
    int hsv_h_sensitivity_MR = 9; // 5 for big green led and 24 for most other
    int hsv_h_low_MR         = hsv_h_base_MR - hsv_h_sensitivity_MR; //hsv_h_base - hsv_h_sensitivity;
    int hsv_h_upper_MR       = hsv_h_base_MR + hsv_h_sensitivity_MR;//hsv_h_base + hsv_h_sensitivity;
    int hsv_s_low_MR         = 135; //100;
    int hsv_s_upper_MR       = 255;
    int hsv_v_low_MR         = 98; //100;
    int hsv_v_upper_MR       = 255;
};
