#pragma once

#include <queue>
#include <C5G/Grasp.h>

namespace APC{
  struct Order{
    std::string object;
    int bin[2];
    C5G::Grasp grasp;
    double score;
  };

  bool operator<(const Order& left, const Order& right);

  typedef std::priority_queue<Order> OrderStatus;

}

