//
// Created by hejia on 8/15/19.
//

#ifndef WECOOK_FIXEDFRAME_H
#define WECOOK_FIXEDFRAME_H

#include <dart/dynamics/FixedFrame.hpp>

namespace wecook {
 class FixedFrame : public dart::dynamics::FixedFrame {
  public:
   FixedFrame(dart::dynamics::Frame *refFrame, const Eigen::Isometry3d &transform) : dart::dynamics::FixedFrame(refFrame, transform) {

   }

   const std::string& setName(const std::string& name) override {

   }

   const std::string& getName() const override {

   }


 };
}

#endif //WECOOK_FIXEDFRAME_H
