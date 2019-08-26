//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveActionExecutor.h"

using namespace wecook;

void PrimitiveActionExecutor::execute(std::shared_ptr<PrimitiveActionNode> &pan) {
  // execute primitive action node
  pan->execute();

  pan->setIfExecuted(true);
}