#include "Stereopsis.hpp"

Stereopsis::Stereopsis(std::string calibration_path)
{
  load_calibration(std::string calibration_path)
}

void Stereopsis::set_left_point(float x, float y)
{
  left_point<Vec2d>(0)[0] = x;
  left_point<Vec2d>(0)[1] = y;
}

void Stereopsis::set_left_point(float x, float y)
{
  right_point<Vec2d>(0)[0] = x;
  right_point<Vec2d>(0)[1] = y;
}

std::vector<float> Stereopsis::calculate_3D_point()
{
  Mat point_3D(1, 1, CV_64FC4);
  point_3D = triangulatePoints(proj_l,proj_r,left_point,right_point,point_3D);
  std::vector<float> result_vec = {point_3D<Vec2d>(0)[0],
                                   point_3D<Vec2d>(0)[1],
                                   point_3D<Vec2d>(0)[2]}
  return result_vec;
}
