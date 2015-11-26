#include <Gripper/PoseFactory.h>

namespace Gripper{
  namespace PoseFactory{
    std::vector<GraspPose> generatePosesOnLine(size_t level, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& planeVector, int toolNumber){
      std::vector<GraspPose> result;

      Eigen::Vector3d center=(p1+p2)/2;
      Eigen::Vector3d line=p2-p1;
      Eigen::Vector3d step=line/level;

      Eigen::Vector3d zAxis=line.cross(planeVector);

      for(int i=0; i<level; ++i){
        Eigen::Vector3d pt=p1+step*i;
        result.emplace_back(std::array<bool,3>{{true, true, true}}, std::array<Eigen::Vector3d, 3>{{planeVector, line, zAxis}}, pt, (pt-center).norm(), toolNumber);
      }

      return result;
    }

    std::vector<GraspPose> generatePosesOnPlane(size_t level, const Eigen::Vector3d& origin, const Eigen::Vector3d& width, const Eigen::Vector3d& height, int toolNumber){
      std::vector<GraspPose> result;

      Eigen::Vector3d center=origin+(width+height)/2;

      Eigen::Vector3d stepW=width/level;
      Eigen::Vector3d stepH=height/level;

      Eigen::Vector3d zAxis=width.cross(height).normalized();

      for(int i=0; i<level; ++i){
        for(int j=0; j<level; ++j){
          Eigen::Vector3d pt=origin+stepW*i+stepH*j;
          result.emplace_back(std::array<bool,3>{{false,false,true}}, std::array<Eigen::Vector3d,3>{{{},{},zAxis}}, pt, (pt-center).norm(), toolNumber);
        }
      }

      return result;
    }
  }
}
