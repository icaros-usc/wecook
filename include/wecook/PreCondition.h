//
// Created by hejia on 8/13/19.
//

#ifndef WECOOK_PRECONDITION_H
#define WECOOK_PRECONDITION_H

class PreCondition {
 public:
  PreCondition() {

  }

  virtual bool isSatisfied() = 0;

};

#endif //WECOOK_PRECONDITION_H
