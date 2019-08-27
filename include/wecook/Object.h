//
// Created by hejia on 7/31/19.
//

#ifndef WECOOK_OBJECT_H
#define WECOOK_OBJECT_H

#include <string>
#include <vector>
#include <dart/dynamics/BodyNode.hpp>

namespace wecook {

class Object {
 public:
  Object(const std::string &name,
         const std::string &url,
         const std::vector<double> &pose) : m_name(name), m_url(url), m_pose(pose) {

  }

  std::string getUrl() const {
    return m_url;
  }

  std::vector<double> getPose() const {
    return m_pose;
  }

  std::string getName() const {
    return m_name;
  }

 private:
  std::string m_name;
  std::string m_url;
  std::vector<double>  m_pose;

  dart::dynamics::BodyNodePtr m_bodyNode = nullptr;
};

}

#endif //WECOOK_OBJECT_H
