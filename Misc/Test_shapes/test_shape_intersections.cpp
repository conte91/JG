#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Geometry>
#include <Gripper/Shape.h>
#include <Gripper/Sphere.h>
#include <Gripper/Cuboid.h>

int main(){

  using Eigen::Translation3d;
  using Eigen::Vector3d;
  using Eigen::Affine3d;
  using Eigen::AngleAxisd;
  using Gripper::Shape;
  using Gripper::Cuboid;
  using Gripper::Sphere;
  using std::vector;

  std::cout << "Here is the (computed!) intersection data relative to Misc/Intersections.fcstd\n";

  vector<Sphere> sp={
    Sphere{Affine3d{Translation3d{0,0,2}}, 4.5},
    Sphere{Affine3d{Translation3d{0,2,0}*AngleAxisd(75*M_PI/180.0, Vector3d{0,55,83}.normalized())}, 20.0},
    Sphere{Affine3d{Translation3d{5,5,12}}, 5.4}
  };

  vector<Cuboid> c={
    Cuboid{Affine3d{Translation3d{0,0,4}*AngleAxisd(50*M_PI/180.0, Vector3d{44,89,13}.normalized())}, 5, 2.4, 10},
    Cuboid{Affine3d{Translation3d{0,1,-2}*AngleAxisd(30*M_PI/180.0, Vector3d{2,0,1}.normalized())}, 10,5,20}
  };

  std::cout << "\n#################\n\nSphere-to-sphere: \n\n";
  for(size_t i=0; i<sp.size(); ++i){
    for(size_t j=0; j<sp.size(); ++j){
      //std::cout << "Obj1 position: " << sp[i].getPose().translation() << "\n";
      //std::cout << "Obj2 position: " << sp[j].getPose().translation() << "\n";
      std::cout << "sp" << i+1 << "sp" << j+1 << ": " << sp[i].getIntersectionVolume(sp[j]) << "\n";
    }
  }

  std::cout << "\n#################\n\nCube-to-sphere: \n\n";
  for(size_t i=0; i<c.size(); ++i){
    for(size_t j=0; j<sp.size(); ++j){
      //std::cout << "Obj1 position: " << c[i].getPose().translation() << "\n";
      //std::cout << "Obj2 position: " << sp[j].getPose().translation() << "\n";
      std::cout << "c" << i+1 << "sp" << j+1 << ": " << c[i].getIntersectionVolume(sp[j]) << "\n";
    }
  }

  std::cout << "\n#################\n\nSphere-to-cube: \n\n";
  for(size_t i=0; i<sp.size(); ++i){
    for(size_t j=0; j<c.size(); ++j){
      //std::cout << "Obj1 position: " << sp[i].getPose().translation() << "\n";
      //std::cout << "Obj2 position: " << c[j].getPose().translation() << "\n";
      std::cout << "sp" << i+1 << "c" << j+1 << ": " << sp[i].getIntersectionVolume(c[j]) << "\n";
    }
  }

  std::cout << "\n#################\n\nCube-to-cube: \n\n";
  for(size_t i=0; i<c.size(); ++i){
    for(size_t j=0; j<c.size(); ++j){
      //std::cout << "Obj1 position: " << c[i].getPose().translation() << "\n";
      //std::cout << "Obj2 position: " << c[j].getPose().translation() << "\n";
      std::cout << "c" << i+1 << "c" << j+1 << ": " << c[i].getIntersectionVolume(c[j]) << "\n";
    }
  }

  return 0;
}
