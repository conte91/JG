#include <APC/Order.h>

namespace APC {
  bool operator<(const Order& left, const Order& right){
    return (left.grasp.score < right.grasp.score);
  }
  const double Order::MIN_SCORE_WE_CAN_MANAGE=0.5;
}
