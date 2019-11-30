//
// Created by hejia on 11/29/19.
//

#ifndef WECOOK_KINECT_H
#define WECOOK_KINECT_H

#include "ai/sensor/camera/Camera.h"

namespace wecook {
namespace ai {
namespace sensor {
namespace camera {

class Kinect {
  void measure(const Eigen::Affine3d &tf);

 protected:

};

}
}
}
}

#endif //WECOOK_KINECT_H
