#pragma once
#include <string>
#include "Pose.h"
namespace C5G{

  struct Grasp{
    std::string object;
    Pose approach;
    Pose grasp;
    double score;
    double forceMin;
    double forceMax;
  };

}
