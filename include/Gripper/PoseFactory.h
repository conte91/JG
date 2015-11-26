#pragma once
#include <Eigen/Core>
#include <Gripper/GraspPose.h>

namespace Gripper{
  namespace PoseFactory{
    std::vector<GraspPose> generatePosesOnLine(size_t level, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& planeVector, int toolNumber);
    std::vector<GraspPose> generatePosesOnPlane(size_t level, const Eigen::Vector3d& origin, const Eigen::Vector3d& width, const Eigen::Vector3d& height, int toolNumber);
  }

}
