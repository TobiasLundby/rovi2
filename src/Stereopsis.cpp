#include "Stereopsis.hpp"

Stereopsis::Stereopsis(std::string calibration_file)
{
  calibrationFile = calibration_file;
  load_calibration(std::string calibration_file)
}

void Stereopsis::set_left_point(float x, float y)
{
  left_point<Vec2d>(0)[0] = x;
  left_point<Vec2d>(0)[1] = y;
}

void Stereopsis::set_right_point(float x, float y)
{
  right_point<Vec2d>(0)[0] = x;
  right_point<Vec2d>(0)[1] = y;
}

std::vector<float> Stereopsis::calculate_3D_point()
{
  Mat left_point(1, 1, CV_64FC2);
  Mat right_point(1, 1, CV_64FC2);
  Mat point_3D(1, 1, CV_64FC4);
  left_point<Vec2d>(0)[0] = left_x;
  left_point<Vec2d>(0)[1] = left_y;
  right_point<Vec2d>(0)[0] = right_x;
  right_point<Vec2d>(0)[1] = right_y;

  point_3D = triangulatePoints(proj_l,proj_r,left_point,right_point,point_3D);
  std::vector<float> result_vec = {point_3D<Vec2d>(0)[0],
                                   point_3D<Vec2d>(0)[1],
                                   point_3D<Vec2d>(0)[2]}
  return result_vec;
}

void Stereopsis::calculate_projection_matrices()
{ // From exercise main file
  KA_l = stereoPair.cam1.intrinsic;
  KA_r = stereoPair.cam2.intrinsic;
  Mat H_l = stereoPair.cam1.transformation;
  Mat H_r = stereoPair.cam2.transformation;

  Mat tmp = Mat::zeros(3,1,CV_64F);
  hconcat(KA_l,tmp,KA_l);
  proj_l = KA_l * H_l;

  hconcat(KA_r,tmp,KA_r);
  proj_r = KA_r * H_r;
}

int load_calibration()
{   // Mostly from exercise
    ifstream ifs(calibrationFile.c_str());
  if (ifs) {
      //Load calibration file
      readStereoCameraFile(calibrationFile, stereoPair);
  } else {
      cout << "Error opening calibration file. Calibration file must be in old OpenCV format. Calibration file must be in old OpenCV format.." << endl;
      return -1;
}

bool Stereopsis::readStereoCameraFile()
{ // From exercise
    int number_of_cameras;
    Camera cam1, cam2;
    std::ifstream ifs(calibrationFile.c_str());
    if (ifs) {
        ifs >> number_of_cameras;
        if (number_of_cameras == 2) {
            loadCamFromStream(ifs, cam1);
            loadCamFromStream(ifs, cam2);
            stereoPair.cam1 = cam1;
            stereoPair.cam2 = cam2;
            return true;
        }
    }
    return false;
}

void loadCamFromStream(std::istream & input, Camera &cam)
{   // From exercise
    Mat intrinsic = Mat::zeros(3, 3, CV_64F);
    Mat distortion = Mat::zeros(4, 1, CV_64F);
    Mat projection = Mat::zeros(4, 4, CV_64F);
    Mat transformation = Mat::eye(4, 4, CV_64F);
    Mat translation = Mat::zeros(3, 1, CV_64F);
    Mat rotation = Mat::zeros(3, 3, CV_64F);
    double image_width, image_height;
    input.precision(20);
    input >> image_width >> image_height;

    input >> intrinsic.at<double>(0, 0) >> intrinsic.at<double>(0, 1)
            >> intrinsic.at<double>(0, 2);
    input >> intrinsic.at<double>(1, 0) >> intrinsic.at<double>(1, 1)
            >> intrinsic.at<double>(1, 2);
    input >> intrinsic.at<double>(2, 0) >> intrinsic.at<double>(2, 1)
            >> intrinsic.at<double>(2, 2);

    input >> distortion.at<double>(0, 0) >> distortion.at<double>(1, 0)
            >> distortion.at<double>(2, 0) >> distortion.at<double>(3, 0);

    input >> rotation.at<double>(0, 0) >> rotation.at<double>(0, 1)
            >> rotation.at<double>(0, 2);
    input >> rotation.at<double>(1, 0) >> rotation.at<double>(1, 1)
            >> rotation.at<double>(1, 2);
    input >> rotation.at<double>(2, 0) >> rotation.at<double>(2, 1)
            >> rotation.at<double>(2, 2);
    input >> translation.at<double>(0) >> translation.at<double>(1)
            >> translation.at<double>(2);

    hconcat(rotation, translation, transformation);
    Mat row = Mat::zeros(1, 4, CV_64F);
    row.at<double>(0, 3) = 1;
    transformation.push_back(row);

    Mat tmp = intrinsic;
    Mat tmp1 = Mat::zeros(3, 1, CV_64F);
    hconcat(tmp, tmp1, tmp);
    projection = tmp * transformation;

    cam.distortion = distortion;
    cam.intrinsic = intrinsic;
    cam.projection = projection;
    cam.transformation = transformation;
    cam.image_height = image_height;
    cam.image_width = image_width;
    cam.translation = translation;
    cam.rotation = rotation;
}
