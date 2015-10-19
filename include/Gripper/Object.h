#pragma once
#include <Gripper/Grip.h>
namespace Gripper{
  class Object{
    public:
      Object();
      Object(const std::unique_ptr<Shape>&, const std::vector<Grip>&);

    private:
      std::unique_ptr<Shape> _myShape;
      std::vector<Grip> _myGrips;
      Gripper readFrom(const cv::FileStorage& fs);

  };
}

namespace cv{

  void write( FileStorage& fs, const std::string& name, const Gripper::Object& model){
    model.writeTo(name, fs);
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
