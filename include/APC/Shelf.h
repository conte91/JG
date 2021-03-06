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
      /** Size of the shelf in #bins */
      static constexpr unsigned int HEIGHT=4;
      static constexpr unsigned int WIDTH=3;
      static const double BIN_DEPTH;
      static const double SECURITY_DISTANCE;
      static const Pose CAMERA_POSE;
      static const Pose POSE_FOR_THE_PHOTOS;
      static const Pose BIN0;

      static Pose getBinPose(int i, int j);

      static Pose getBinSafePose(int i, int j);

      static constexpr const auto& getBinCornerPose=getBinPose;
      static constexpr const auto& getBinCenterPose=getBinSafePose;
  };
}
