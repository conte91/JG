#pragma once
#include <string>
#include "Pose.h"
namespace C5G{

  struct Grasp{
    std::string object;
    int row;
    int column;
    Pose approach;
    Pose grasp;
    // REMOVED double forceMin;
    double forceMax;
    double score;
  };

}
