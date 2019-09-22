//
// Created by hejia on 9/19/19.
//

#ifndef WECOOK_METAACTUATEINFO_H
#define WECOOK_METAACTUATEINFO_H

#include "Action.h"

namespace wecook {
class MetaActuateInfo {
 public:
  MetaActuateInfo(const Action &action) : m_action(action) {

  }

  MetaActuateInfo(const MetaActuateInfo& other) {
    m_action = other.getAction();
  }

  Action getAction() const {
    return m_action;
  }

 private:
  Action m_action;
};
}

#endif //WECOOK_METAACTUATEINFO_H
