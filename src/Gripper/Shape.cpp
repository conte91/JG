#include <Gripper/Shape.h>
#include <pcl/point_types.h>

namespace Gripper{
  bool Shape::knowsHowToIntersect(const Shape& s) const {
    for(const auto& x: getKnownIntersections()){
      if(x==s.getID() || x=="Anything"){
        return true;
      }
    }
    return false;
  }


  auto Shape::getKnownIntersections() const -> KnownIntersections {
    return {};
  }

  std::string Shape::getID() const {
    return "Generic";
  }

  double Shape::getIntersectionVolume(const Shape& s) const {
    if(knowsHowToIntersect(s)){
      return intersectionVolume(s, BASE_APPROX_LEVEL);
    }
    assert(s.knowsHowToIntersect(*this) && "No valid intersections found between me and the other shape!!");
    return s.intersectionVolume(*this, BASE_APPROX_LEVEL);
  }
  double Shape::intersectionVolume(const Shape& s, size_t level) const {
    double myVol=getVolume();
    double otherVol=s.getVolume();

    if(myVol > otherVol){
      auto cubes=s.getCubettiVolume(BASE_APPROX_LEVEL);
      return (countContainedPoints(cubes)*otherVol)/cubes.cols();
    }
    else{
      auto cubes=getCubettiVolume(BASE_APPROX_LEVEL);
      return (s.countContainedPoints(cubes)*myVol)/cubes.cols();
    }
    return -1;
  }
  const std::vector<double>& Shape::getDimensions() const {
    return _dimensions;
  }

  Eigen::Matrix<double, 4, Eigen::Dynamic> Shape::getCubettiVolume(size_t level) const {
    assert(false && "Empty shape cannot be transformed into point clouds.. This is probably a bug in the shape building process");
    return Eigen::Matrix<double, 4, Eigen::Dynamic>{};
  }

  Eigen::Matrix<double, 4, Eigen::Dynamic> Shape::getCubettiSurface(size_t level) const {
    assert(false && "Empty shape cannot be transformed into point clouds.. This is probably a bug in the shape building process");
    return Eigen::Matrix<double, 4, Eigen::Dynamic>{};
  }

  Shape::PointsPtr Shape::getPCVolume(size_t level) const 
  {
    auto points=getCubettiVolume(level);
    PointsPtr result(new Points);
    for(int i=0; i<points.cols(); ++i){
      auto p=points.col(i).cast<float>();
      result->push_back(PointType{p[0],p[1],p[2]});
    }
    return result;
  }

  Shape::PointsPtr Shape::getPCSurface(size_t level) const
  {
    auto points=getCubettiSurface(level);
    PointsPtr result(new Points);
    for(int i=0; i<points.cols(); ++i){
      auto p=points.col(i).cast<float>();
      result->push_back(PointType{p[0],p[1],p[2]});
    }
    return result;
  }

  Shape::Shape(const RelPose& pose, const std::vector<double>& dims)
    :
      _pose(pose),
      _dimensions(dims)
  {
  }

  Shape::RelPose Shape::getPose() const {
    return _pose;
  }

  size_t Shape::countContainedPoints(const Shape::PointsMatrix& pt) const {
    assert(false && "Empty shape called.. This is probably a bug in your shape tree");
    return 0;
  }

  double Shape::getVolume() const{
    assert(false && "Empty shape volume called.. This is probably a bug in your shape tree");
    return 0;
  }

  Shape::~Shape(){
  }

  Shape operator*(const Eigen::Affine3d& lhs, const Shape& rhs){
    Shape result=rhs;
    result._pose=lhs*rhs._pose;
    return result;
  }
}
