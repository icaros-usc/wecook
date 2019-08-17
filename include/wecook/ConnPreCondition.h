//
// Created by hejia on 8/16/19.
//

#ifndef WECOOK_CONNPRECONDITION_H
#define WECOOK_CONNPRECONDITION_H

#include <Eigen/Core>
#include <dart/dynamics/MetaSkeleton.hpp>

#include "PreCondition.h"
#include "ContainingMap.h"

class ConnPreCondition : public PreCondition {
 public:
  ConnPreCondition(std::shared_ptr<wecook::ContainingMap> &containingMap, const std::string &containerName, const std::string &containedName, bool shouldConn) :
      m_containingMap(containingMap), m_containerName(containerName), m_containedName(containedName), m_shouldConn(shouldConn) {

  }

  bool isSatisfied() {
    return m_shouldConn == m_containingMap->hasTuple(m_containerName, m_containedName);
  }

 private:
  std::shared_ptr<wecook::ContainingMap> m_containingMap;
  std::string m_containerName;
  std::string m_containedName;
  bool m_shouldConn;
};

#endif //WECOOK_CONNPRECONDITION_H
