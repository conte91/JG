#include <C5G/Pose.h>
#include <iostream>

std::ostream& operator<< (std::ostream& os, const C5G::Pose& x){
  os << "{("<< x.x << "," << x.y << "," << x.z << ");(" << x.alpha << "," << x.beta << "," << x.gamma << ")}" ;
}

namespace C5G{

  Pose Pose::operator+(const Pose& o) const{
    /** TODO tmp */
    //std::cout << "Computing this stuff: " << *this << "+" << o << "=";
    Pose xx={x+o.x, y+o.y, z+o.z, alpha, beta, gamma /* TODO compute w/ angless */};
    //std::cout << xx << "\n";
    return xx;
  }
}
