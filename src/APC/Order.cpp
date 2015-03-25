#include <APC/Order.h>

namespace APC {
  bool operator<(const Order& left, const Order& right){
    return (left.grasp.score < right.grasp.score);
  }

  Order::Order(const std::string& objName){
    object=objName;
    bin[0]=0;
    bin[1]=0;
  }
  const double Order::MIN_SCORE_WE_CAN_MANAGE=0.5;
}
