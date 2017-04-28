#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>


struct Camera {
    Mat intrinsic;
    Mat transformation;
    Mat distortion;
    Mat projection;
    Mat translation;
    Mat rotation;
    double image_width;
    double image_height;

    void printData() {
        cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
                << distortion << "\n" << transformation << "\n" << projection
                << endl;
    }
};

struct StereoPair {
    Camera cam1;
    Camera cam2;
};

class Stereopsis {

public:
  Stereopsis(std::string)
  void set_left_point(float,float);
  void set_right_point(float,float);
  std::vector<float> calculate_3D_point();
  ~Stereopsis();
private:
  load_calibration(std::string);
  bool readStereoCameraFile();
  void loadCamFromStream(std::istream & input, Camera &cam)

  std::string calibrationFile;
  StereoPair stereoPair;
  Mat proj_l;
  Mat proj_r;

};
