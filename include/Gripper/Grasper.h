#pragma once

#include <map>
#include "Shape.h"
#include "GraspPose.h"

namespace Gripper{

  typedef std::map<std::string, std::pair<std::shared_ptr<const Shape>, double> > ShapeDatabase;
  typedef std::map<std::string, std::vector<GraspPose> > GraspDatabase;
  typedef std::pair<std::string,  Eigen::Affine3d> ObjWithPose;

  typedef std::vector<ObjWithPose> ObjectsScene;

}
