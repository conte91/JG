#include <algorithm>
#include <Gripper/Cuboid.h>

namespace Gripper{
  std::string Cuboid::getID() const {
    return "Cuboid";
  }
  auto Cuboid::getKnownIntersections() const -> KnownIntersections{
    return {"Cuboid", "Sphere"};
  }

  double Cuboid::getVolume() const{
    return _dimensions[0]*_dimensions[1]*_dimensions[2];
    return 0;
  }

  Cuboid::PointsMatrix Cuboid::getCubettiSurface(size_t level) const {
    double W=_dimensions[0];
    double H=_dimensions[1];
    double D=_dimensions[2];

    /** Get an approximation using points of this cube size 1/level */
    double stepX=W/level;
    double stepY=H/level;
    double stepZ=D/level;

    const size_t expectedPts=6*level*level+2;
    Eigen::Matrix<double, Eigen::Dynamic, 4> cubePoints(expectedPts, 4);

    size_t pos=0;

    /** Work on Z axis, we will add caps later */
    for(size_t i=0; i<=level; ++i){
      double zCoord=i*stepZ;

      /** Left and right borders */
      for(size_t j=0; j<=level; ++j){
        double yCoord=j*stepY;
        cubePoints.row(pos++)=Eigen::Vector4d{0, yCoord, zCoord , 1}.transpose();
        cubePoints.row(pos++)=Eigen::Vector4d{W, yCoord, zCoord , 1}.transpose();
      }

      /** Up and down borders */
      for(size_t j=1; j<level; ++j){
        double xCoord=j*stepX;
        cubePoints.row(pos++)=Eigen::Vector4d{xCoord, 0, zCoord , 1}.transpose();
        cubePoints.row(pos++)=Eigen::Vector4d{xCoord, H, zCoord , 1}.transpose();
      }
    }

    /** Now add caps */
    for(size_t i=1; i<level; ++i){
      for(size_t j=1; j<level; ++j){
        double xCoord=i*stepX;
        double yCoord=j*stepY;
        cubePoints.row(pos++)=Eigen::Vector4d{xCoord, yCoord, 0, 1}.transpose();
        cubePoints.row(pos++)=Eigen::Vector4d{xCoord, yCoord, D, 1}.transpose();
      }
    }
    assert(pos==(expectedPts));
    return (_pose*cubePoints.transpose());
  }

  Cuboid::PointsMatrix Cuboid::getCubettiVolume(size_t level) const {
    double W=_dimensions[0];
    double H=_dimensions[1];
    double D=_dimensions[2];

    /** Get an approximation using points of this cube size 1/level */
    double stepX=W/level;
    double stepY=H/level;
    double stepZ=D/level;
    double startX=stepX/2.0;
    double startY=stepY/2.0;
    double startZ=stepZ/2.0;

    Eigen::Matrix<double, Eigen::Dynamic, 4> cubePoints(level*level*level, 4);

    size_t pos=0;
    for(size_t i=0; i<level; ++i){
      for(size_t j=0; j<level; ++j){
        for(size_t k=0; k<level; ++k){
          cubePoints.row(pos++)=Eigen::Vector4d{startX+i*stepX, startY+j*stepY, startZ+k*stepZ, 1}.transpose();
        }
      }
    }
    return (_pose*cubePoints.transpose());
  }

  /*
  double Cuboid::intersectionVolume(const Shape& s, size_t level) const{
    auto id=s.getID();
    if(id=="Sphere"){
      auto cubePoints=getCubettiMatrix(_dimensions);
      auto realCubePoints=_pose.matrix().topRows<3>()*cubePoints;
      auto distances=Eigen::Vector3d{1,1,1}.transpose()*((realCubePoints-s.getPose().translation().rowwise().replicate(level*level*level)).cwiseAbs2());

      double R=s.getDimensions()[0];
      auto bom=R*R-distances.array();
      double V=(bom > 0.0).count()*(W()*H()*D()/(level*level*level));
      return V;
    }
    assert(id=="Cuboid");
    return Shape::intersectionVolume(s,level);
  }*/

  size_t Cuboid::countContainedPoints(const Cuboid::PointsMatrix& pt) const{
    auto realOtherPoints=(_pose.inverse().matrix().topRows<3>()*pt).array();
    auto d=_dimensions;
    auto ltdPoints=(realOtherPoints.row(0) < d[0]) && (realOtherPoints.row(1) < d[1]) && (realOtherPoints.row(2) < d[2]);
    auto greatPoints=(realOtherPoints.row(0) > 0) && (realOtherPoints.row(1) > 0) && (realOtherPoints.row(2) > 0);
    return (ltdPoints && greatPoints).count();
  }

  double Cuboid::W() const {
    return _dimensions[0];
  }
  double Cuboid::H() const { 
    return _dimensions[1];
  }
  double Cuboid::D() const {
    return _dimensions[2];
  }
  Cuboid::Cuboid(Eigen::Affine3d pose, double W, double H, double D)
    :
      Shape(pose, {W,H,D})
  {
  }

  Cuboid::~Cuboid(){
  }
}

