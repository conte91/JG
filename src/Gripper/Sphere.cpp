#include <cmath>
#include <Gripper/Sphere.h>

namespace Gripper{
  double Sphere::getVolume() const {
    return _dimensions[0]*_dimensions[0]*_dimensions[0]*M_PI*4.0/3.0;
  }

  double Sphere::intersectionVolumeHeuristic(const Shape& s) const{
    using std::max;
    using std::min;
    assert(s.getID()=="Sphere" && "Don't know heuristic");
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

  size_t Sphere::countContainedPoints(const Shape::PointsMatrix& pt) const {
    auto distanceFromS=pt.topRows<3>()-_pose.translation().replicate(1,pt.cols());;
    return (distanceFromS.cwiseAbs2().colwise().sum().array() < _dimensions[0]).count();
  }
  
  double Sphere::haveNoIntersection(const Shape& s) const{
    assert(s.getID()=="Sphere" && "don't know noIntersection heuristic");
    using std::max;
    using std::min;
    double r1=_dimensions[0];
    double r2=s.getDimensions()[0];
    double R=max(r1, r2);
    double r=min(r1, r2);
    Eigen::Vector3d center1=_pose.translation();
    Eigen::Vector3d center2=s.getPose().translation();
    double d=(center1-center2).norm();

    return (d>R+r);
  }

  auto Sphere::noIntersectionHeuristic() const -> ShapeList {
    return {"Sphere"};
  }
  auto Sphere::intersectionHeuristic() const -> ShapeList {
    return {"Sphere"};
  }

  std::string Sphere::getID() const {
    return "Sphere";
  }

  Eigen::Matrix<double, 4, Eigen::Dynamic> Sphere::getCubettiVolume(size_t level) const {

    /** Get an approximation using points of this cube size 1/level */
    double stepX=2.0/level;
    double stepY=2.0/level;
    double stepZ=2.0/level;
    double startX=-1.0+stepX/2.0;
    double startY=-1.0+stepY/2.0;
    double startZ=-1.0+stepZ/2.0;

    Eigen::Matrix<double, Eigen::Dynamic, 4> cubePoints(level*level*level, 4);

    size_t pos=0;
    for(size_t i=0; i<level; ++i){
      for(size_t j=0; j<level; ++j){
        for(size_t k=0; k<level; ++k){
          double x=startX+i*stepX, y=startY+j*stepY, z=startZ+k*stepZ;
          if(x*x+y*y+z*z<=1.0){
            cubePoints.row(pos++)=Eigen::Vector4d{x,y,z,1}.transpose();
          }
        }
      }
    }

    cubePoints.conservativeResize(pos, Eigen::NoChange);
    Eigen::Matrix4d mySize;

    double r=_dimensions[0];
    mySize << 
      r, 0, 0, 0,
      0, r, 0, 0,
      0, 0, r, 0,
      0, 0, 0, 1;


    return (_pose*mySize*cubePoints.transpose());

  }

  Sphere::Sphere(const RelPose& pose, double R)
    :
      Shape(pose, {R})
  {
  }

  Sphere::PointsMatrix Sphere::getCubettiSurface(size_t level) const {
    /** No equal  density is required, let's go with UV coordinates */
    double stepLat=2*M_PI/level;
    double stepLon=2*M_PI/level;
 
    double r=_dimensions[0];

    PointsMatrix result(4,level*(level-1)+2);
    size_t pos=0;
    for(size_t i=1; i<level; ++i){
      double lat=stepLat*i;
      double z=r*cos(lat);
      for(size_t j=0; j<level; ++j){
        double lon=stepLon*j;
        double x=r*sin(lat)*cos(lon), y=r*sin(lat)*sin(lon);
        result.col(pos++)=Eigen::Vector4d{x,y,z,1};
      }
    }
    result.col(pos++)=Eigen::Vector4d{0,0,r,1};
    result.col(pos++)=Eigen::Vector4d{0,0,-r,1};
    return _pose*result;
  }
  Shape* Sphere::clone() const {
    return new Sphere(*this);
  }

}
