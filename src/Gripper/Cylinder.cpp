#include <cmath>

#include <Eigen/Geometry>
#include <Gripper/Cylinder.h>

namespace Gripper{
  Cylinder::Cylinder(Eigen::Affine3d pose, double R, double H)
    :
      Shape(pose, {R,H})
      {
      }
  Cylinder::~Cylinder()
  {
  }

  Cylinder::PointsMatrix Cylinder::getCubettiSurface(size_t level) const{
    size_t pos=0;

    double H=_dimensions[1];
    double R=_dimensions[0];
    double stepZ=H/level;
    double stepR=R/level;
    double stepAngle=2*M_PI/level;

    size_t expectedSize=3*level*level-level+2;
    PointsMatrix result(4, expectedSize);
    /** On height */
    for(size_t i=0; i<=level; ++i){
      double z=stepZ*i;
      for(size_t j=0; j<level; ++j){
        double alpha=j*stepAngle;
        double x=R*cos(alpha), y=R*sin(alpha);
        result.col(pos++)=Eigen::Vector4d{x,y,z,1};
      }
    }

    /** Now add caps */
    for(size_t i=1; i<level; ++i){
      double r=stepR*i;
      for(size_t j=0; j<level; ++j){
        double alpha=j*stepAngle;
        double x=r*cos(alpha), y=r*sin(alpha);
        result.col(pos++)=Eigen::Vector4d{x,y,0,1};
        result.col(pos++)=Eigen::Vector4d{x,y,H,1};
      }
    }

    /** Center of caps, just for beautiness */
    result.col(pos++)=Eigen::Vector4d{0,0,0,1};
    result.col(pos++)=Eigen::Vector4d{0,0,H,1};
    assert(pos==expectedSize);
    return _pose*result;

  }

  Cylinder::PointsMatrix Cylinder::getCubettiVolume(size_t level) const{
    /** Get an approximation using points of this cube size 1/level */
    double stepX=2.0/level;
    double stepY=2.0/level;
    double stepZ=1.0/level;
    double startX=-1.0+stepX/2.0;
    double startY=-1.0+stepY/2.0;
    double startZ=-1.0+stepZ/2.0;

    Eigen::Matrix<double, Eigen::Dynamic, 4> cubePoints(level*level*level, 4);

    size_t pos=0;
    for(size_t i=0; i<level; ++i){
      for(size_t j=0; j<level; ++j){
        for(size_t k=0; k<level; ++k){
          double x=startX+i*stepX, y=startY+j*stepY, z=startZ+k*stepZ;
          if(x*x+y*y<1.0){
            cubePoints.row(pos++)=Eigen::Vector4d{x,y,z,1}.transpose();
          }
        }
      }
    }

    cubePoints.conservativeResize(pos, Eigen::NoChange);
    Eigen::Matrix4d mySize;

    double r=_dimensions[0];
    double H=_dimensions[1];
    mySize << 
      r, 0, 0, 0,
      0, r, 0, 0,
      0, 0, H, 0,
      0, 0, 0, 1;
    return (_pose*mySize*cubePoints.transpose());
  }
  std::string Cylinder::getID() const{
    return "Cylinder";
  }
  double Cylinder::getVolume() const{
    double r=_dimensions[0], h=_dimensions[1];
    return r*r*M_PI*h;
  }

  double Cylinder::intersectionVolume(const Shape& s, size_t level) const {
    return Shape::intersectionVolume(s,level);
  }

  Cylinder::KnownIntersections Cylinder::getKnownIntersections() const {
    return {"Anything"};
  }

  size_t Cylinder::countContainedPoints(const PointsMatrix& pt) const {
    auto myFramePts=(_pose.inverse().matrix()*pt).topRows<3>().array();
    auto d=_dimensions;
    auto ltdPoints=(myFramePts.row(2) < d[2] ) && (myFramePts.row(2) > 0);
    auto inCirclePoints=myFramePts.cwiseAbs2().colwise().sum() < d[0]*d[0];
    return (ltdPoints && inCirclePoints).count();
  }

}

