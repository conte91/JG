#pragma once
#include "Shape.h"
#include "GraspPose.h"

namespace Gripper{
  struct Object{
    public:
      typedef std::vector<GraspPose> GraspSet;

      Object();
      Object(const std::shared_ptr<const Shape>&, const GraspSet&);

      Shape::Ptr myShape;
      GraspSet myGrasps;
      static Object readFrom(const cv::FileNode& fs);

  };
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Gripper::Object& model);
  void read(const FileNode& node, Gripper::Object& x, const Gripper::Object& default_value);
}
