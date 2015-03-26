#include <C5G/Pose.h>
#include <APC/Shelf.h>

namespace APC {

  using C5G::Pose;

  const Pose Shelf::POSE(0.4, 0.5, -0.7, 0, 0, 0);
  const double Shelf::HEIGHT=4;
  const double Shelf::WIDTH=3;
  const double Shelf::BIN_HEIGHT=0.3;
  const double Shelf::BIN_WIDTH=0.3;
  const double Shelf::BIN_DEPTH=0.3;
  const double Shelf::SECURITY_DISTANCE=0.05;
  const Pose Shelf::BIN0(0, 0.15, 0.90, 0, 0, 0);

  Pose Shelf::getBinPose(int i, int j){
    return Shelf::BIN0+Pose(0, -i*BIN_WIDTH, j*BIN_HEIGHT, 0, 0, 0);
  }

  Pose Shelf::getBinSafePose(int i, int j){
    return getBinPose(i, j)+Pose(-Shelf::SECURITY_DISTANCE, 0, 0, 0, 0, 0);
  }

}
