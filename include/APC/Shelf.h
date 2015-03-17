#pragma once

#include <C5G/Pose.h>

namespace APC{

  class Shelf{
    private:
      Shelf();
      ~Shelf();
    public:
      typedef C5G::Pose Pose;
      static const Pose POSE;
      static const double BIN_HEIGHT;
      static const double BIN_WIDTH;
      static const double BIN_DEPTH;
      static const double SECURITY_DISTANCE;
      static const Pose BIN0;

      static Pose getBinPose(int i, int j);

      static Pose getBinSafePose(int i, int j);
  };
}
