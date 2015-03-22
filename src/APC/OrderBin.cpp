#include <APC/OrderBin.h>
#include <C5G/Pose.h>

namespace APC{
  const C5G::Pose OrderBin::POSE={0, 0, 0, 0, 0, 0}; /** Relative to the CENTER of the bin */
  const double OrderBin::HEIGHT=0.5;
  const double OrderBin::WIDTH=0.5;
  const double OrderBin::DEPTH=0.5;
}
