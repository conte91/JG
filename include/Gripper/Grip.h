#pragma once
#include <Eigen/Geometry>
namespace Gripper{
  struct Grip{
    Eigen::Affine3d approach;
    Eigen::Affine3d grip;
    double forceMin;
    double forceMax';
  };
}
