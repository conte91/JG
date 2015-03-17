#include <C5G/Pose.h>

namespace C5G{
  Pose Pose::operator+(const Pose& o) const{
    /** TODO tmp */
    Pose xx={x+o.x, y+o.y, z+o.z, alpha, beta, gamma /* TODO compute w/ angless */};
    return xx;
  }
}
