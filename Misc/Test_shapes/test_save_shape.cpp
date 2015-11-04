#include <Gripper/Shape.h>
#include <Gripper/Sphere.h>
#include <Gripper/Cuboid.h>
#include <Gripper/ComposedShape.h>
#include <Gripper/Cylinder.h>
#include <Gripper/ShapeBuilder.h>
int main(){

  using Gripper::Shape;
  using Gripper::Cuboid;
  using Gripper::Sphere;
  using Gripper::Cylinder;
  using Gripper::ComposedShape;
  using std::make_shared;
  using cv::FileStorage;
  using Eigen::Affine3d;
  using Eigen::Translation3d;
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;

  Sphere sp{Affine3d{Translation3d{0,0,2}}, 3};

  Cuboid c{Affine3d{Translation3d{0,0,-1}*AngleAxisd(50*M_PI/180.0, Vector3d{44,89,13}.normalized())}, 1, 1.4, 3};

  Cylinder cy{Affine3d{Translation3d{0,2,0}*AngleAxisd(M_PI/2, Vector3d{0,1,0})}, 0.5, 1.5};

  std::vector<std::shared_ptr<const Shape> > components{make_shared<decltype(sp)>(sp), make_shared<decltype(c)>(c)};
  ComposedShape comp{Affine3d{Translation3d{0,-1,0}*AngleAxisd(45*M_PI/180.0, Vector3d{10,11,0}.normalized())}, components};

  Shape* data[4]={&sp, &c, &cy, &comp};

  std::string names[4]={"sphere.yml", "cuboid.yml", "cylinder.yml", "compound.yml"};

  for(int i=0; i<4; ++i){
    FileStorage fs(names[i], FileStorage::WRITE);
    fs << "shape" << *(data[i]);
  }
  return 0;
}
