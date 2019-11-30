//
// Created by hejia on 11/29/19.
//

#include <iostream>

#include "ai/sensor/camera/kinect/SimKinect.h"

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " model_file.obj" << "dot_path" << std::endl;
    exit(-1);
  }

  ::wecook::ai::sensor::Camera::Params cam_params;

  cam_params.m_w = 640;
  cam_params.m_h = 480;
  cam_params.m_cx = 320;
  cam_params.m_cy = 240;
  cam_params.m_z_near = 0.5;
  cam_params.m_z_far = 6.0;
  cam_params.m_fx = 580;
  cam_params.m_fy = 580;
  cam_params.m_tx = 0.075;

  ::wecook::ai::sensor::camera::kinect::SimKinect camera{cam_params, argv[1], argv[2]};

  Eigen::Affine3d tf{};
  camera.measure(tf);
}