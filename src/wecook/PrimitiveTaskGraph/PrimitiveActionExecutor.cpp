//
// Created by hejia on 8/26/19.
//

#include "wecook/ContainingMap.h"
#include "wecook/PrimitiveTaskGraph/PrimitiveActionExecutor.h"
#include "wecook/ObjectMgr.h"

using namespace wecook;

void PrimitiveActionExecutor::execute(std::shared_ptr<PrimitiveActionNode> &pan) {
  // execute primitive action node
  pan->execute(m_agents, m_objMgr, m_containingMap);
}