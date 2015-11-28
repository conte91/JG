#include <Gripper/Object.h>
#include <Gripper/PoseFactory.h>
#include <Utils/CvStorage.h>

namespace Gripper{
  Object::Object()
  {
  }

  Object::Object(const std::shared_ptr<const Shape>& shape, const GraspSet& grips)
    :
      myShape(shape),
      myGrasps(grips)
  {
  }


  Object Object::readFrom(const cv::FileNode& fs){

    Shape::Ptr s;
    GraspSet g;

    bool autoPoses;

    fs["shape"] >> s;
    fs["autoPoses"] >> autoPoses;

    /** TODO  */
#if 0
    if(autoPoses){
      /* Automatically generates a set of poses from the shape */
      g+=generatePosesFromShape(s);
    }
#endif

    for(const auto& x : fs["grasps"]){

      std::string type;
      x["type"] >> type;

      const auto& data=x["data"];

      if(type=="custom"){
        GraspPose customGrasp;
        data >> customGrasp;
        g.push_back(customGrasp);
      }
      else if(type=="line"){
        int toolN;
        int level;
        data["toolN"] >> toolN;
        Eigen::Vector3d p1, p2, planeVector;
        data["p1"] >> p1;
        data["p2"] >> p2;
        data["planeVector"] >> planeVector;
        data["level"] >> level;
        auto newPoses=PoseFactory::generatePosesOnLine(level,p1,p2,planeVector,toolN);
        g.insert(g.end(), newPoses.begin(), newPoses.end());
      }
      else if(type=="Plane"){
        int toolN;
        int level;
        data["toolN"] >> toolN;
        Eigen::Vector3d origin, width, height;
        data["origin"] >> origin;
        data["width"] >> width;
        data["height"] >> height;
        data["level"] >> level;
        auto newPoses=PoseFactory::generatePosesOnPlane(level,origin, width, height, toolN);
        g.insert(g.end(), newPoses.begin(), newPoses.end());
      }
      else if(type=="Cuboid"){
        int toolN;
        int level;
        data["toolN"] >> toolN;
        Eigen::Matrix4d pose;
        double width, height, depth;
        data["pose"] >> pose;
        data["width"] >> width;
        data["height"] >> height;
        data["depth"] >> depth;
        data["level"] >> level;
        auto newPoses=PoseFactory::generatePosesOnCuboid(level, Eigen::Affine3d{pose}, width, height, depth, toolN);
        g.insert(g.end(), newPoses.begin(), newPoses.end());
      }
    }
    return Object{s,g};

  }
}



namespace cv{

  void write( FileStorage& fs, const std::string& name, const Gripper::Object& model){
    //model.writeTo(name, fs);
    assert(false && "TODO");
  }
  void read(const FileNode& node, Gripper::Object& x, const Gripper::Object& default_value){
    if(node.empty()){
      x=default_value;
    }
    else{
      x=Gripper::Object::readFrom(node);
    }
  }

}
