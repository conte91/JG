#include <C5G/Pose.h>
#include <iostream>

std::ostream& operator<< (std::ostream& os, const C5G::Pose& x){
  os << "{("<< x.x << "," << x.y << "," << x.z << ");(" << x.alpha << "," << x.beta << "," << x.gamma << ")}" ;
  return os;
}

namespace C5G{

  Pose Pose::whichIsRelativeTo(const Pose& origin) const {
    //std::cout << "getSQReferenceFrame\n";
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    /** Apply Euler angle rotations */
    transformation.rotate (Eigen::AngleAxisf (alpha, Eigen::Vector3f::UnitX()));
    transformation.rotate (Eigen::AngleAxisf (beta, Eigen::Vector3f::UnitY()));
    transformation.rotate (Eigen::AngleAxisf (gamma, Eigen::Vector3f::UnitZ()));
    transformation.translation() << x, y, z;

    transformation.rotate (Eigen::AngleAxisf (origin.alpha, Eigen::Vector3f::UnitX()));
    transformation.rotate (Eigen::AngleAxisf (origin.beta, Eigen::Vector3f::UnitY()));
    transformation.rotate (Eigen::AngleAxisf (origin.gamma, Eigen::Vector3f::UnitZ()));
    transformation.translation() << origin.x, origin.y, origin.z;

    return transform2Pose(transformation);
  }

  Pose Pose::transform2Pose(const Eigen::Affine3f& x){
    /** Implements the refined algorithm from extracting euler angles from rotation matrix
     * see Mike Day, Insomniac Games, "Extracting Euler Angles from a Rotation Matrix"
     */
    auto transl=x.translation();
    auto m=x.rotation();

    double theta1=atan2(m(1,2),m(2,2));
    double c2=hypot(m(0,0),m(0,1));
    double theta2=atan2(-m(0,2),c2);
    double s1=sin(theta1);
    double c1=cos(theta1);
    double theta3=atan2(s1*m(2,0)-c1*m(1,1),c1*m(1,1)-s1*m(2,1));
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
