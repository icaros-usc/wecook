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

inline std::vector<size_t> findAllOccurances(const std::string &data, const char &toSearch) {
  std::vector<size_t> ret;
  size_t pos = data.find(toSearch);

  while (pos != std::string::npos) {
    ret.push_back(pos);

    pos = data.find(toSearch, pos + 1);
  }
  return ret;
}
}

#endif //WECOOK_STRUTILS_H
