//
// Created by hejia on 8/26/19.
//

#include "wecook/PrimitiveGrabNode.h"
#include "wecook/utils.h"

using namespace wecook;

void PrimitiveGrabNode::execute(std::map<std::string, std::shared_ptr<Agent>) {
  // TODO 
  m_grabPose->mT0_w = m_refBodyNode->getTransform();

}
