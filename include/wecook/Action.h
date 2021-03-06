//
// Created by hejia on 7/26/19.
//

#ifndef WECOOK_ACTION_H
#define WECOOK_ACTION_H

#include <string>
#include <vector>

namespace wecook {

class Action {
 public:
  Action(const std::vector<std::string> &pids,
         const std::vector<std::string> &location,
         const std::vector<std::string> &ingredients,
         const std::string &verb,
         const std::string &tool)
      : m_pids(pids), m_ingredients(ingredients), m_locations(location), m_tool(tool), m_verb(verb) {

  }

  Action() {

  }

  Action(const Action &other) {
    m_pids = other.get_pids();
    m_ingredients = other.get_ingredients();
    m_locations = other.get_locations();
    m_tool = other.get_tool();
    m_verb = other.get_verb();
  }

  std::string get_verb() const {
    return m_verb;
  }

  std::vector<std::string> get_locations() const {
    return m_locations;
  }

  std::string get_tool() const {
    return m_tool;
  }

  std::vector<std::string> get_ingredients() const {
    return m_ingredients;
  }

  std::vector<std::string> get_pids() const {
    return m_pids;
  }

 private:
  std::vector<std::string> m_pids;
  std::vector<std::string> m_locations;
  std::string m_tool;
  std::vector<std::string> m_ingredients;
  std::string m_verb;
};
}

#endif //WECOOK_ACTION_H
