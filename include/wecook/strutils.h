//
// Created by hejia on 8/7/19.
//

#ifndef WECOOK_STRUTILS_H
#define WECOOK_STRUTILS_H

#include <string>
#include <vector>

namespace wecook {
inline bool strin(const std::string &str, const std::vector<std::string> &strs) {
  for (const auto &var : strs) {
    if (str == var) {
      return true;
    }
  }
  return false;
}
}

#endif //WECOOK_STRUTILS_H
