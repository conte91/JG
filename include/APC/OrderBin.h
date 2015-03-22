#pragma once

#include <C5G/Pose.h>

namespace APC{

  class OrderBin{
    private:
      OrderBin();
      ~OrderBin();
    public:
      typedef C5G::Pose Pose;
      static const Pose POSE; /** Relative to the CENTER of the bin */
      static const double HEIGHT;
      static const double WIDTH;
      static const double DEPTH;
  };
}
