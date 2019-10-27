//
// Created by hejia on 10/26/19.
//

#ifndef WECOOK_LOGICFORMULA_H
#define WECOOK_LOGICFORMULA_H

#include <z3++.h>
#include "PDDLProblem.h"
#include "PDDLDomain.h"

namespace wecook {
namespace ai {
namespace planner {
namespace taskPlanner {
/*! Logic Formula class, used for represent pddl task planning problem, and be solved by SMT solver. */
class LogicFormula {
 public:
  LogicFormula();
};
}
}
}
}

#endif //WECOOK_LOGICFORMULA_H
