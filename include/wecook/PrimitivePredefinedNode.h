//
// Created by hejia on 8/24/19.
//

#ifndef WECOOK_PRIMITIVEPREDEFINEDNODE_H
#define WECOOK_PRIMITIVEPREDEFINEDNODE_H

#include <aikido/constraint/dart/TSR.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include "PrimitiveActionNode.h"

namespace wecook {

class PrimitivePredefinedNode : public PrimitiveActionNode {
 public:
  PrimitivePredefinedNode(const std::string &pid, bool ifHead = false) : PrimitiveActionNode(pid,
                                                                                             "predefined",
                                                                                             ifHead) {

  }

 private:

};

}

#endif //WECOOK_PRIMITIVEPREDEFINEDNODE_H
