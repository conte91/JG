#include <C5G/Pose.h>
#include <iostream>

std::ostream& operator<< (std::ostream& os, const C5G::Pose& x){
  os << "{("<< x.x << "," << x.y << "," << x.z << ");(" << x.alpha << "," << x.beta << "," << x.gamma << ")}" ;
}

namespace C5G{

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

  Pose::Pose(){
  }

  Pose Pose::operator+(const Pose& o) const{
    /** TODO tmp */
    return Pose(x+o.x, y+o.y, z+o.z, alpha, beta, gamma /* TODO compute w/ angless */);
  }

}
