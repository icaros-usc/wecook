//
// Created by hejia on 8/26/19.
//

#ifndef WECOOK_PRIMITIVEACTIONEXECUTOR_H
#define WECOOK_PRIMITIVEACTIONEXECUTOR_H

#include "PrimitiveActionNode.h"

namespace wecook {

  class PrimitiveActionExecutor {
   public:
    PrimitiveActionExecutor() = default;

    void execute(std::shared_ptr<PrimitiveActionNode> &pan);
  };

}

#endif //WECOOK_PRIMITIVEACTIONEXECUTOR_H
