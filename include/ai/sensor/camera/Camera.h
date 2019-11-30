//
// Created by hejia on 11/29/19.
//

#ifndef WECOOK_CAMERA_H
#define WECOOK_CAMERA_H

#include "ai/sensor/Sensor.h"

namespace wecook {
namespace ai {
namespace sensor {

 class Camera : public ::wecook::ai::Sensor {
  enum NOISE_TYPE {
    GAUSSIAN = 0,
    PERLIN,
    SIMPLEX,
    NONE,
  };

 public:
  struct Params;

  Camera() = default;

  void measure(const Eigen::Affine3d &tf);
};

struct Camera::Params {
  int m_w, m_h;
  double m_z_near, m_z_far;
  double m_fx, m_fy;
  double m_cx, m_cy;
  double m_tx;

  NOISE_TYPE m_noise = NONE;
};

}
}
}

#endif //WECOOK_CAMERA_H
