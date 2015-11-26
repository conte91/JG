#include <Gripper/GraspPose.h>

namespace Gripper{
  GraspPose::GraspPose(const decltype(constraints)& cconstraints , const decltype(axis)& aaxis, const decltype(pickPose)& ppickpose, decltype(preferenceScore) ppreferenceScore, decltype(toolNumber) ttoolNumber)
    :
      constraints(cconstraints),
      axis(aaxis),
      pickPose(ppickpose),
      preferenceScore(ppreferenceScore),
      toolNumber(ttoolNumber)
  {
  }
  GraspPose GraspPose::constrain(const std::function<double(Eigen::Affine3d&)>& f){
    bool c0=constraints[0], c1=constraints[1], c2=constraints[2];
    if((c0 && c1) || (c1 && c2) || (c2 && c0) ){
      /** No further optimizations available as the three main directional axis are already fixed */
      return *this;
    }
    /* TODO TODO TODO */
    return *this;
  }
  bool GraspPose::operator<(const GraspPose& other){
    return (preferenceScore < other.preferenceScore);
  }

  GraspPose::GraspPose()
  {
  }
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const GraspPose& pose){
  }
  void read(const FileNode& node, GraspPose& x, const GraspPose& default_value){
    if(node.empty()){
      x=default_value;
      return;
    }
    std::array<bool, 3> c;
    node["constraints"] >> c;

    std::array<Eigen::Vector3d, 3> axis;
    node["axis"] >> axis;

    Eigen::Vector3d pose;
    node["pose"] >> pose;

    double score;
    node["score"] >> score;

    int n;
    node["nTool"] >> n;

    x=GraspPose{c, axis, pose, score, n};

  }
}

