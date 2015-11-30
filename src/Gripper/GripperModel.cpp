#include <Utils/CvStorage.h>
#include <Gripper/Grasper.h>
#include <Gripper/GripperModel.h>
#include <Gripper/Shape.h>
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
  std::pair<double, Eigen::Affine3d> GripperModel::getBestGrasp(const std::string& name, const ObjectsScene& scene, const ObjectDB& objDB){
    auto poses=objDB.at(name).myGrasps;
    std::sort(poses.begin(), poses.end());

    double currentBestScore=std::numeric_limits<double>::max();

    Eigen::Affine3d bestGrasp;

    /** Intersect the gripper with all the (other) objects into the scene  -- iterating for each pose and for each object after this imposes that as soon as a valid pose is taken, it is the best one (doing it the other way would mean that the other objects have to be scanned fully) */
    for(const auto& currentPose : poses){

      std::cout << "Analyzing pose with Z axis:\n" << currentPose.axis[2]<< "\nPick position:\n" << currentPose.pickPose << "\n";
      assert(currentPose.constraints[2] && "Object not constrained over Z axis!");

      /** Iterate over all the possible items of the same name */ 
      for(const auto& object : scene){

        if(object.first!=name){
          continue;
        }

        /** Relative to the gripper's local frame */
        auto toolPose=_toolPoses[currentPose.toolNumber];
        std::cout << "Tool pose: \n" << toolPose.matrix() << "\n";
        std::cout << "Base pose: \n" << _basePose.matrix() << "\n";

        /** We want to minimize at our best the difference in rotation from a fixed frame (base) in the gripper's reference and the identity */
        Eigen::Affine3d transformFromToolToBase=toolPose.inverse()*_basePose;
        Eigen::Affine3d transformFromBaseToTool=transformFromToolToBase.inverse();

        /** Fitted transformation of the tool 
            This is relative to the object's frame */
        Eigen::Affine3d fittedToolPose;

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

          /** Build rotation matrix in object's coordinates from the 3 axes */
          fittedToolPose.linear().matrix() << myConstraints[0], myConstraints[1], myConstraints[2];
          fittedToolPose.linear().matrix() = fittedToolPose.linear().matrix().inverse();
          fittedToolPose.translation().matrix() << currentPose.pickPose;
        }
        else{
          /** Desired alignment of the base reference on the gripper in GLOBAL coordinates */
          Eigen::Affine3d wantedBaseAlignment = Eigen::Affine3d::Identity();

          /** How this transforms into object's space: base frame, in object coordinates, transformed into tool's space */
          Eigen::Affine3d idealToolPose = transformFromBaseToTool*object.second.inverse()*wantedBaseAlignment;

          std::cout << "We would like our tool to be aligned to \n" << idealToolPose.matrix() << "\n";

          /** Find the rotation of the ideal tool's Z axis and align it to the constrained one */
          Eigen::Vector3d idealToolZAxis = idealToolPose.linear()*Eigen::Vector3d{0,0,1};
          Eigen::Vector3d realToolZAxis = currentPose.axis[2];
          std::cout << "\n\nidealToolZAxis: \n" << idealToolZAxis << "\n\nrealToolZAxis:\n" << realToolZAxis.matrix() << "\n";
          Eigen::Vector3d axis;
          double angle;
          if(idealToolZAxis.isApprox(realToolZAxis)){
            std::cout << "Wow already aligned\n";
            angle=0;
            axis=Eigen::Vector3d::UnitX(); /** Dummy */
          }
          else if(idealToolZAxis.isApprox(-realToolZAxis)){
            angle=M_PI;
            axis=Eigen::Vector3d::UnitX(); /** Dummy */
          }
          else{
            /** Find angle-axis rotation */
            axis = idealToolZAxis.cross(realToolZAxis).normalized();
            angle = ::acos(idealToolZAxis.dot(realToolZAxis));
          }

          /** Align the two axis */
          Eigen::Affine3d alignment(Eigen::AngleAxisd(angle, axis));
          std::cout << "Angle: " << angle <<"\nAxis:\n" << axis << "\n";
          std::cout << "\nMultiplication c*i:\n" << (alignment.linear()*idealToolZAxis).matrix() << "\n";
          fittedToolPose.linear()=alignment.linear()*idealToolPose.linear();
          fittedToolPose.translation().matrix() << currentPose.pickPose;
          assert((fittedToolPose.linear()*Eigen::Vector3d::UnitZ()).isApprox(realToolZAxis));
          std::cout << "fitted tool pose: \n" << fittedToolPose.matrix()  << "\n" ;
        }

        /** Pose of the base of the gripper in object's coordinate frames */
        /** We want to solve gripperPose(in object)*ToolPose(in gripper)=fittedToolPose(in object)
            -> gripperPose(in object) = fittedToolPose*ToolPose^(-1)(in gripper)*/
        Eigen::Affine3d armPose{fittedToolPose*toolPose.inverse()};

        std::cout << "Arm pose in object coordinates: \n" << armPose.matrix() << "\n";

        /** Gripper's shape in object's frame */
        Shape::Ptr myShapeInPose=armPose*_myShape;

        double score=0;

        /** Now, intersect the gripper with anyone in the world */
        for(const auto& otherObject : scene){
          /** Transform all the scene into objects' coordinate frames */
          Eigen::Affine3d otherObjectPose=object.second.inverse()*otherObject.second;
          auto otherObjectShape=otherObjectPose*(objDB.at(otherObject.first).myShape);

          /** Compute intersection with an obejct of the scene and apply score function and mobility coefficient */
          score+=scoreFunction(myShapeInPose->getIntersectionVolume(*otherObjectShape))*objDB.at(otherObject.first).myMobility;
        }

        if(score < ScoreParams::THRESHOLD_NO_INTERSECTION){
          std::cout << "\n\n\n\nArmPose:\n" << armPose.matrix() << "object pose: \n" << object.second.matrix() << "\n";
          return std::make_pair(score, object.second*armPose);
        }

        if(score < currentBestScore){
          currentBestScore=score;
          /** In global coordinates */
          bestGrasp=object.second*armPose;
        }
      }
    }
    return std::make_pair(currentBestScore,bestGrasp);
  }
  Shape::Ptr GripperModel::shape(){
    return _myShape;
  }

  GripperModel::GripperModel(){
  }

  GripperModel::GripperModel(const std::vector<Eigen::Affine3d> poses, Eigen::Affine3d base, const Shape::Ptr& shape)
    :
    _toolPoses(poses),
    _basePose(base),
    _myShape(shape)
  {
  }
}

namespace cv{
  void read(const FileNode& node, Gripper::GripperModel& x, const Gripper::GripperModel& default_value) {
    if(node.empty()){
      x=default_value;
      return;
    }
    else{
      const auto& tools=node["tools"];
      std::vector<Eigen::Affine3d> poses;
      for(const auto& t : tools){
        Eigen::Matrix4d value;
        t >> value;
        poses.push_back(Eigen::Affine3d{value});
      }
      Gripper::Shape::Ptr shape;
      node["shape"] >> shape;
      Eigen::Matrix4d base;
      node["base"] >> base;

      x=Gripper::GripperModel{poses,Eigen::Affine3d{base},shape};
    }
  }
}
