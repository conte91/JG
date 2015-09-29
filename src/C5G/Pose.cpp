#include <C5G/Pose.h>
#include <Eigen/Core>
#include <iostream>

std::ostream& operator<< (std::ostream& os, const C5G::Pose& x){
  os << "{("<< x.x << "," << x.y << "," << x.z << ");(" << x.alpha << "," << x.beta << "," << x.gamma << ")}" ;
  return os;
}

namespace C5G{

  Eigen::Affine3d Pose::poseToTransform(const Pose& x){
    Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
    transformation.translation() << x.x, x.y, x.z;
    Eigen::Affine3d rotation = Eigen::Affine3d::Identity();
    /** Apply Euler angle rotations */
    rotation.rotate (Eigen::AngleAxisd (x.alpha, Eigen::Vector3d::UnitX()));
    rotation.rotate (Eigen::AngleAxisd (x.beta, Eigen::Vector3d::UnitY()));
    rotation.rotate (Eigen::AngleAxisd (x.gamma, Eigen::Vector3d::UnitZ()));
    transformation.linear()=rotation.linear();
    return transformation;
  }

  Eigen::Affine3d Pose::toTransform() const {
    return poseToTransform(*this);
  }

  Pose Pose::andIWantItRelativeTo(const Pose& origin) const {
    auto& transformation=poseToTransform(origin)*poseToTransform(*this);
    return transform2Pose(transformation);
  }

  Pose Pose::whichIsRelativeTo(const Pose& origin) const {
    auto& transformation=poseToTransform(origin)*poseToTransform(*this);
    std::cout <<"\n" <<  transformation.translation() << "\nR\n" << transformation.rotation() << "\n";
    return transform2Pose(transformation);
  }

  Pose Pose::transform2Pose(const Eigen::Affine3d& x){
    /** Implements the refined algorithm from extracting euler angles from rotation matrix
     * see Mike Day, Insomniac Games, "Extracting Euler Angles from a Rotation Matrix"
     */
    auto transl=x.translation();
    auto m=x.rotation();

    double theta1=atan2(m(1,2),m(2,2));
    //std::cout << "Theta1: " << theta1 << "\n";
    double c2=hypot(m(0,0),m(0,1));
    //std::cout << "c2: " << c2 << "\n";
    double theta2=atan2(-m(0,2),c2);
    //std::cout << "Theta2: " << theta2 << "\n";
    double s1=sin(theta1);
    //std::cout << "s1: " << s1 << "\n";
    double c1=cos(theta1);
    //std::cout << "c1: " << c1 << "\n";
    double theta3=atan2(s1*m(2,0)-c1*m(1,0),c1*m(1,1)-s1*m(2,1));
    //std::cout << "Theta3: " << theta3 << "\n";
    return C5G::Pose(transl[0], transl[1], transl[2], theta1, theta2, theta3);
  }

  Pose::Pose()
  {
    this->x=0;
    this->y=0;
    this->z=0;
    this->alpha=0;
    this->beta=0;
    this->gamma=0;
  }

  Pose::Pose(/* In meters */ double x, double y, double z, /* In radians */ double alpha, double beta, double gamma, /* Override default units */bool useMMDeg){
    static const double RAD_TO_DEG=57.2957795130824;
    if(useMMDeg){
      x*=1000;
      y*=1000;
      z*=1000;
      alpha*=RAD_TO_DEG;
      beta*=RAD_TO_DEG;
      gamma*=RAD_TO_DEG;
    }
    this->x=x;
    this->y=y;
    this->z=z;
    this->alpha=alpha;
    this->beta=beta;
    this->gamma=gamma;
  }

  Pose Pose::operator+(const Pose& o) const{
    /** TODO tmp */
    return Pose(x+o.x, y+o.y, z+o.z, alpha, beta, gamma /* TODO compute w/ angless */);
  }

}
