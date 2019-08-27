//
// Created by hejia on 8/26/19.
//

#ifndef WECOOK_PRIMITIVEACTIONEXECUTOR_H
#define WECOOK_PRIMITIVEACTIONEXECUTOR_H

#include "PrimitiveActionNode.h"

namespace wecook {

class PrimitiveActionExecutor {
 public:
  PrimitiveActionExecutor(std::map<std::string, std::shared_ptr<Agent>> &agents,
                          std::shared_ptr<ObjectMgr> &objMgr,
                          std::shared_ptr<ContainingMap> &containingMap)
      : m_agents(agents), m_objMgr(objMgr), m_containingMap(containingMap) {

  }

  void execute(std::shared_ptr<PrimitiveActionNode> &pan);

 private:
  std::map<std::string, std::shared_ptr<Agent>> m_agents;
  std::shared_ptr<ObjectMgr> m_objMgr;
  std::shared_ptr<ContainingMap> m_containingMap
};

}

#endif //WECOOK_PRIMITIVEACTIONEXECUTOR_H
