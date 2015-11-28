#include <Gripper/GraspPose.h>
#include <Utils/CvStorage.h>

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
  
  void GraspPose::drawToViewer(pcl::visualization::PCLVisualizer& viewer) const{
    assert(constraints[2] && "Not constrained over Z axis :|");
    {
      std::stringstream myTitle;
      myTitle << "Grasp Z from " << pickPose[0] << "," << pickPose[1] << "," << pickPose[2] << " with axis " << axis[2][0] << "," << axis[2][1] << "," << axis[2][2];
      pcl::PointXYZ p1, p2;
      p1.getVector3fMap()=pickPose.cast<float>();
      p2.getVector3fMap()=(pickPose+axis[2]).cast<float>();
      viewer.addArrow(p1,p2,0,0,255,myTitle.str());
    }

    if(constraints[0]){
      std::stringstream myTitle;
      myTitle << "Grasp X from " << pickPose[0] << "," << pickPose[1] << "," << pickPose[2] << " with axis " << axis[0][0] << "," << axis[0][1] << "," << axis[0][0];
      pcl::PointXYZ p1, p2;
      p1.getVector3fMap()=pickPose.cast<float>();
      p2.getVector3fMap()=(pickPose+axis[0]).cast<float>();
      viewer.addArrow(p1,p2,255,0,0,myTitle.str());
    }
    if(constraints[1]){
      std::stringstream myTitle;
      myTitle << "Grasp Y from " << pickPose[0] << "," << pickPose[1] << "," << pickPose[2] << " with axis " << axis[1][0] << "," << axis[1][1] << "," << axis[1][0];
      pcl::PointXYZ p1, p2;
      p1.getVector3fMap()=pickPose.cast<float>();
      p2.getVector3fMap()=(pickPose+axis[1]).cast<float>();
      viewer.addArrow(p1,p2,0,255,0,myTitle.str());
    }
  }
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Gripper::GraspPose& pose){
    assert(false && "TODO");
  }
  void read(const FileNode& node, Gripper::GraspPose& x, const Gripper::GraspPose& default_value){
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

    x=Gripper::GraspPose{c, axis, pose, score, n};

  }
}

