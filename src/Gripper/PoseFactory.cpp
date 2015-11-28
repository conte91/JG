#include <Gripper/PoseFactory.h>
#include <iterator>

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
    std::vector<GraspPose> generatePosesOnCuboid(size_t level, const Eigen::Affine3d& pose, double width, double height, double depth, int toolNumber){
      constexpr double margin=0.1;
      constexpr double reduceSize=1.0-2*margin;

      std::vector<GraspPose> result;
      Eigen::Vector3d origin=pose*Eigen::Vector3d::Zero();
      Eigen::Vector3d xDir=pose*Eigen::Vector3d::UnitX();
      Eigen::Vector3d yDir=pose*Eigen::Vector3d::UnitY();
      Eigen::Vector3d zDir=pose*Eigen::Vector3d::UnitZ();

      Eigen::Vector3d xSide=xDir*width;
      Eigen::Vector3d ySide=yDir*height;
      Eigen::Vector3d zSide=zDir*depth;

      Eigen::Vector3d xSideRed=xSide*reduceSize;
      Eigen::Vector3d ySideRed=ySide*reduceSize;
      Eigen::Vector3d zSideRed=zSide*reduceSize;

      Eigen::Vector3d xMargin=margin*xSide;
      Eigen::Vector3d yMargin=margin*ySide;
      Eigen::Vector3d zMargin=margin*zSide;

      std::array<std::vector<GraspPose>, 6> srcs;
      /** Generate poses for each of the 6 planes of the cube */

      /** Bottom and top face */
      srcs[0]=generatePosesOnPlane(level, origin+xMargin+yMargin, xSideRed, ySideRed, toolNumber);
      srcs[1]=generatePosesOnPlane(level, origin+zSide-xMargin-yMargin, ySideRed, xSideRed, toolNumber);


      /** Left and right face */
      srcs[2]=generatePosesOnPlane(level, origin+yMargin+zMargin, ySideRed, zSideRed, toolNumber);
      srcs[3]=generatePosesOnPlane(level, origin+xSide-yMargin-zMargin, zSideRed, ySideRed, toolNumber);

      /** Front and rear */
      srcs[4]=generatePosesOnPlane(level, origin+xMargin+zMargin, zSideRed, xSideRed, toolNumber);
      srcs[5]=generatePosesOnPlane(level, origin+ySide-xMargin-zMargin, xSideRed, zSideRed, toolNumber);

      /** Sums them all (well, moves them all) */
      for(const auto& x : srcs){
        result.insert(result.end(), std::make_move_iterator(x.begin()), std::make_move_iterator(x.end()));
      }

      return result;
    }
  }
}
