#pragma once

#include <utility>
#include "Types.h"
#include "GraspPose.h"
#include "Grasper.h"
#include "Shape.h"

namespace Gripper{
  class GripperModel{
    public:
      GripperModel(const std::vector<Eigen::Affine3d> poses, Eigen::Affine3d base, const Shape::Ptr& shape);
      GripperModel();
      std::pair<double, Eigen::Affine3d> getBestGrasp(const std::string& name, const ObjectsScene& scene, const ObjectDB& objects);
      Shape::Ptr shape();

    private:
      struct ScoreParams{
          static constexpr double VMAX=10.0e-6;
          static constexpr double VEASY=1.0e-6;
          static constexpr double ALPHA=10.0;
          static constexpr double K=5.0;
          static constexpr double THRESHOLD_NO_INTERSECTION=1e-09;
      };
      std::vector<Eigen::Affine3d> _toolPoses;

      /** This is the pose which we would like to align perfectly to the identity when constraining grasps */
      Eigen::Affine3d _basePose;

      Shape::Ptr _myShape;
      static double scoreFunction(double intVol);
  };
}

namespace cv{
  void read(const FileNode& node, Gripper::GripperModel& x, const Gripper::GripperModel& default_value=Gripper::GripperModel{} );
}
