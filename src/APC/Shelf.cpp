#include <C5G/Pose.h>
#include <APC/Shelf.h>
#include <cmath>

namespace APC {

  using C5G::Pose;

  /** Pose of the front-left corner of the shelf relative to the robot */
  const Pose Shelf::POSE(0.792, 0.454, 0.836, 0, 0, 0);

  /** Size of a single bin */
  const double Shelf::BIN_HEIGHT=0.3;
  const double Shelf::BIN_WIDTH=0.3;
  const double Shelf::BIN_DEPTH=0.3;

  /** Distance from the front of a bin to the safe position (margin) */
  const double Shelf::SECURITY_DISTANCE=0.05;

  /** Position of bin A wrt the origin of the shelf */
  const Pose Shelf::BIN0(0, -0.15, -0.30, 0, 0, 0);

  const Pose Shelf::CAMERA_POSE(1.90, 0, 0, 0, 0, M_PI);

  const Pose Shelf::POSE_FOR_THE_PHOTOS(0.396, 0, 1.320, 0, 0, 0);

  Pose Shelf::getBinPose(int i, int j){
    return (Shelf::BIN0+Pose(0, -i*BIN_WIDTH, -j*BIN_HEIGHT, 0, 0, 0));
  }

  Pose Shelf::getBinSafePose(int i, int j){
    return (getBinPose(i, j)+Pose(-Shelf::SECURITY_DISTANCE, -BIN_WIDTH/2, BIN_HEIGHT/2, 0, 0, 0));
  }

}
