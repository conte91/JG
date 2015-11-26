#include <Gripper/Grasper.h>
#include <limits>

namespace Gripper{
  GraspPose GripperModel::getBestGrasp(const std::string& name, const ObjectsScene& scene, const GraspDatabase& possibleGrasps, const ShapeDatabase& shapes){
    auto poses=possibleGrasps.at(name);
    std::sort(poses.begin(), poses.end());

    double currentBestScore=std::numeric_limits<double>::max();

    Eigen::Affine3d bestGrasp;

    /** Iterate over all the possible items of the same name */ 
    for(size_t i=0; i<scene.size(); ++i){
      if(scene[i]!=name){
        continue;
      }

      const auto& thisObject=scene[i];

      double thisScore;

      /** Intersect the gripper with all the (other) objects into the scene */
      for(const auto& currentPose : poses){
        auto toolPose=_toolPoses[currentPose.toolNumber];
        /** This is already into object refFrame */
        Eigen::Affine3d armPose=toolPose*poses.pose;

        myShapeInPose=armPose*_myShape;

        double score=0;

        for(const auto& x : scene){
          /** Transform all the scene into objects' coordinate frames */
          otherObjectPose=thisObject.second.inverse()*x.second;
          otherObjectShape=otherObjectPose*shapes[x.first].first;

          /** Compute intersection with an obejct of the scene and apply score function and mobility coefficient */
          score+=scoreFunction(myShapeInPose.getIntersectionVolume(otherObjectShape))*shapes[x.first].second;
        }

        if(score < THRESHOLD_NO_INTERSECTION){
          return currentPose;
        }

        if(score < currentBestScore){
          currentBestScore=score;
          bestGrasp=currentPose;
        }
      }
    }
  }
}
