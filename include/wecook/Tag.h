//
// Wecook licence info!
//

#ifndef WECOOK_TAG_H
#define WECOOK_TAG_H

#include <string>
#include <vector>
#include <dart/dynamics/BodyNode.hpp>

namespace wecook {

class Tag {
 public:
  Tag(const uint32_t &tagId,
         const std::string &objectName,
         const std::vector<double> &objectPose) : m_tagId(tagId), m_objectName(objectName), m_objectPose(objectPose) {

  }

  std::string getObjectName() const {
    return m_objectName;
  }

  std::vector<double> getObjectPose() const {
    return m_objectPose;
  }

  uint32_t getTagId() const {
    return m_tagId;
  }

 private:
  uint32_t m_tagId;
  std::string m_objectName;
  std::vector<double>  m_objectPose;

  // dart::dynamics::BodyNodePtr m_bodyNode = nullptr; PK-TODO: I probably don't need this.
};

}

#endif //WECOOK_TAG_H
