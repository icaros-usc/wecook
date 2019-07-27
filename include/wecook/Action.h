//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_ACTION_H
#define WECOOK_ACTION_H

namespace wecook {
class Action {
 public:
  Action(const std::string &location,
         const std::vector<std::string> &ingredients,
         const std::string &verb,
         const std::string &tool) : m_ingredients(ingredients), m_location(location), m_tool(tool), m_verb(verb) {

  }

 private:
  std::string m_location;
  std::string m_tool;
  std::vector<std::string> m_ingredients;
  std::string m_verb;
};
}

#endif //WECOOK_ACTION_H
