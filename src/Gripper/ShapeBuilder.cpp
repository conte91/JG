#include <opencv2/core/core.hpp>
#include <Gripper/Shape.h>
#include <Gripper/Cuboid.h>
#include <Gripper/Sphere.h>
#include <Gripper/Cylinder.h>
#include <Gripper/ComposedShape.h>
#include <Gripper/ShapeBuilder.h>
#include <Utils/Eigen2CV.h>

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Gripper::Shape& model){
    model.writeTo(fs);
  }
  void read(const FileNode& node, std::unique_ptr<Gripper::Shape>& res, const std::unique_ptr<Gripper::Shape>& default_value ){
    using Eigen::Affine3d;
    using Gripper::Shape;

    std::string name;
    std::vector<double> dimensions;

    Eigen::Matrix4d poseE;

    node ["name"] >> name;
    node ["dimensions"] >> dimensions;
    node ["pose"] >> poseE;

    if(name=="Cuboid"){
      assert(dimensions.size()==3);
      res= std::unique_ptr<Gripper::Shape>{new Gripper::Cuboid(Affine3d{poseE}, dimensions[0], dimensions[1], dimensions[2])};
      return;
    }
    else if(name=="Sphere"){
      assert(dimensions.size()==1);
      res= std::unique_ptr<Gripper::Shape>{new Gripper::Sphere(Affine3d{poseE}, dimensions[0])};
      return;
    }
    else if(name=="Cylinder"){
      assert(dimensions.size()==2);
      res= std::unique_ptr<Gripper::Shape>{new Gripper::Cylinder(Affine3d{poseE}, dimensions[0], dimensions[1])};
      return;
    }
    else if(name=="Composed"){
      std::vector<std::shared_ptr<const Shape> > stuff;
      const auto& fn=node["shapes"];
      for(const auto& x : fn){
        std::unique_ptr<Shape> elem;
        x >> elem;
        stuff.emplace_back(std::move(elem));
      }
      res= std::unique_ptr<Gripper::Shape>{new Gripper::ComposedShape(Affine3d{poseE}, stuff)};
      return;
    }
    throw std::string("Invalid shape type!!\n");
  }

}
