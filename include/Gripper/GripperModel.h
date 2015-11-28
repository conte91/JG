#pragma once

#include "Types.h"
#include "GraspPose.h"
#include "Grasper.h"

namespace Gripper{
  class GripperModel{
    public:
        GraspPose getBestGrasp(const std::string& name, const ObjectsScene& scene, const ObjectDB& objects);

    private:
      struct ScoreParams{
          static constexpr double VMAX=10.0e-6;
          static constexpr double VEASY=1.0e-6;
          static constexpr double ALPHA=10.0;
          static constexpr double K=5.0;
          static constexpr double THRESHOLD_NO_INTERSECTION=0.1e-06;
      };
      std::vector<Eigen::Affine3d> _toolPoses;

      /** This is the pose which we would like to align perfectly to the identity when constraining grasps */
      Eigen::Affine3d _basePose;

      std::shared_ptr<const Shape> _myShape;
      static double scoreFunction(double intVol);
  };
}
