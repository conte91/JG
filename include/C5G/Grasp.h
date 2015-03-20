#pragma once
#include <string>
#include "Pose.h"

struct Grasp{
  std::string object;
  C5G::Pose approach;
  C5G::Pose grasp;
  double force;
};
