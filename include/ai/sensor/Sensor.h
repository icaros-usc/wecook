//
// Created by hejia on 11/29/19.
//

#ifndef WECOOK_SENSOR_H
#define WECOOK_SENSOR_H

#include <Eigen/Eigen>

namespace wecook {
namespace ai {

class Sensor {
 public:
  virtual void measure(const Eigen::Affine3d &tf) = 0;
};

}
}

#endif //WECOOK_SENSOR_H
