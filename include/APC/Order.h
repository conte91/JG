#pragma once

#include <queue>
#include <C5G/Grasp.h>

//possibility to complete an order
namespace APC{
  struct Order{
    std::string object;
    int bin[2];
    C5G::Grasp grasp;
    static const double MIN_SCORE_WE_CAN_MANAGE;
    Order(const std::string& object="none");
  };

  bool operator<(const Order& left, const Order& right);

  typedef std::priority_queue<Order> OrderStatus;

  std::ostream& operator<< (std::ostream& out, const Order& in);
  std::ostream& operator<< (std::ostream& out, const OrderStatus& in);
}

