#include <Gripper/Grasper.h>
#include <Gripper/GripperModel.h>
#include <limits>
#include <algorithm>
#include <cmath>

namespace Gripper{
  double GripperModel::scoreFunction(double vInt){
    constexpr double VMAX=ScoreParams::VMAX;
    constexpr double VEASY=ScoreParams::VEASY;
    constexpr double ALPHA=ScoreParams::ALPHA;
    constexpr double K=ScoreParams::K;
    if(vInt >= VMAX){
      return std::numeric_limits<double>::max();
    }
    if(vInt > VEASY){
      return ALPHA*(1-K*(1+(VMAX-VEASY)/(vInt-VMAX)));
    }
    return ALPHA*vInt/VEASY;
  }
  GraspPose GripperModel::getBestGrasp(const std::string& name, const ObjectsScene& scene, const ObjectDB& objects){
    auto poses=objects.at(name).myGrasps;
    std::sort(poses.begin(), poses.end());

    double currentBestScore=std::numeric_limits<double>::max();

    GraspPose bestGrasp;

    /** Iterate over all the possible items of the same name */ 
    for(size_t i=0; i<scene.size(); ++i){
      if(scene[i].first!=name){
        continue;
      }

      const auto& thisObject=scene[i];

      double thisScore;

      /** Intersect the gripper with all the (other) objects into the scene */
      for(const auto& currentPose : poses){
        auto toolPose=_toolPoses[currentPose.toolNumber];

        /** This is already into object refFrame */
        assert(currentPose.constraints[2] && "Object not constrained over Z axis!");

        Eigen::Affine3d transformFromToolToBase=toolPose.inverse()*_basePose;
        Eigen::Affine3d transformFromBaseToTool=transformFromToolToBase.inverse();

        /** Fitted transformation of the tool wrt the gripper's base coordinate system */
        Eigen::Affine3d currentPoseTransform;

        if(std::count(currentPose.constraints.begin(),currentPose.constraints.end(), true)>=2){
          /** Totally constrained */
          std::array<Eigen::Vector3d, 3> myConstraints=currentPose.axis;

          if(currentPose.constraints[0]){
            /** XZ constraint */
            myConstraints[1]=myConstraints[2].cross(myConstraints[0]);
          }
          if(currentPose.constraints[1]){
            myConstraints[0]=myConstraints[1].cross(myConstraints[2]);
          }

          currentPoseTransform.linear().matrix() << myConstraints[0], myConstraints[1], myConstraints[2];
          currentPoseTransform.linear().matrix() = currentPoseTransform.linear().matrix().inverse();
          currentPoseTransform.translation().matrix() << currentPose.pickPose;
        }
        else{
          /** Only the Z axis is constrained, use it */
          Eigen::Affine3d wantedBaseAlignment = Eigen::Affine3d::Identity();
          Eigen::Affine3d idealToolPose = transformFromBaseToTool*wantedBaseAlignment;

          /** Find the rotation of the ideal tool's Z axis and align it to the constrained one */
          Eigen::Vector3d idealToolZAxis = idealToolPose*Eigen::Vector3d{0,0,1};
          Eigen::Vector3d realToolZAxis = currentPose.axis[2];
          if(idealToolZAxis.isApprox(realToolZAxis)){
            currentPoseTransform=idealToolPose;
          }
          else{
            /** Find angle-axis rotation */
            Eigen::Vector3d axis = idealToolZAxis.cross(realToolZAxis);
            double angle = ::acos(idealToolZAxis.dot(realToolZAxis));
            /** Align the two axis */
            currentPoseTransform=Eigen::AngleAxisd(angle, axis)*idealToolPose;
            assert((currentPoseTransform*idealToolZAxis).isApprox(realToolZAxis));
          }
        }

        Eigen::Affine3d armPose{toolPose*currentPoseTransform};

        Shape myShapeInPose=armPose*(*_myShape);

        double score=0;

        for(const auto& x : scene){
          /** Transform all the scene into objects' coordinate frames */
          Eigen::Affine3d otherObjectPose=thisObject.second.inverse()*x.second;
          Shape otherObjectShape=otherObjectPose*(*(shapes.at(x.first).first));

          /** Compute intersection with an obejct of the scene and apply score function and mobility coefficient */
          score+=scoreFunction(myShapeInPose.getIntersectionVolume(otherObjectShape))*shapes.at(x.first).second;
        }

        if(score < ScoreParams::THRESHOLD_NO_INTERSECTION){
          return currentPose;
        }

        if(score < currentBestScore){
          currentBestScore=score;
          bestGrasp=currentPose;
        }
      }
    }
    return bestGrasp;
  }
}
