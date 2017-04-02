#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

class Stereopsis {

public:
  Stereopsis(std::string)
  void set_left_point(float,float);
  void set_right_point(float,float);
  std::vector<float> calculate_3D_point();
  ~Stereopsis();
private:
  load_calibration(std::string);
  Mat left_point(1, 1, CV_64FC2);
  Mat right_point(1, 1, CV_64FC2);
  Mat proj_l;
  Mat proj_r;

};
