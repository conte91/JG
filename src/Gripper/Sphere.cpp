#include <cmath>
#include <Gripper/Sphere.h>

namespace Gripper{
  std::string Sphere::getID() const {
    return "Sphere";
  }
  auto Sphere::getKnownIntersections() const -> KnownIntersections{
    return {"Sphere"};
  }
  double Sphere::intersectionVolume(const Shape& s) const{
    using std::max;
    using std::min;
    assert(s.getID()=="Sphere");
    double r1=_dimensions[0];
    double r2=s.getDimensions()[0];
    double R=max(r1, r2);
    double r=min(r1, r2);
    Eigen::Vector3d center1=_pose.translation();
    Eigen::Vector3d center2=s.getPose().translation();
    double d=(center1-center2).norm();

    if(d>R+r){
      return 0;
    }
    if(d<=R-r){
      /** Small sphere is contained fully into other one */
      return 4.0/3.0*M_PI*r*r*r;
    }
    double V=M_PI*pow(R+r-d, 2)*(d*d+2*d*r-3*r*r+2*d*R+6*r*R-3*R*R)/(12*d);
    return V;
  }

  Sphere::Sphere(const RelPose& pose, double R)
    :
      Shape(pose, {R})
  {
  }

}
