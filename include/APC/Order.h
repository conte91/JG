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
  };

  bool operator<(const Order& left, const Order& right);

  typedef std::priority_queue<Order> OrderStatus;


}

