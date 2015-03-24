#include <APC/Order.h>

namespace APC {
  bool operator<(const Order& left, const Order& right){
    return (left.score < right.score);
  }
}
