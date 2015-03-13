#include <C5G/C5G.h>
#include <APC/Shelf.h>

const Pose Shelf::POSE(0, 0, 1, 0, 0, 0);
const Pose Shelf::POSE(0, 0, 1, 0, 0, 0);
const double Shelf::BIN_HEIGHT=0.2;
const double Shelf::BIN_WIDTH=0.2;
const double Shelf::BIN_DEPTH=0.2;
const Pose Shelf::BIN0(0, 0, 1, 0, 0, 0);

Pose& Shelf::getBinPose(int i, int j){
  return Shelf::BIN0+Pose(0, -i*BIN_WIDTH, j*BIN_HEIGHT, 0, 0, 0);
}

Pose& Shelf::getBinSafePose(int i, int j){
  return getBinPose(i, j)+Pose(-Shelf::BIN_DEPTH/2-Shelf::SECURITY_DISTANCE, 0, 0, 0, 0, 0);
}

