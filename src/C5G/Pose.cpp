#include <C5G/Pose.h>

Pose Pose::operator+(const Pose& o){
  /** TODO tmp */
  return Pose(x+o.x, y+o.y, z+o.z, alpha, beta, gamma /* TODO compute w/ angless */);
}
