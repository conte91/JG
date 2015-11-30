#include <Gripper/Shape.h>
#include <Utils/Eigen2CV.h>
#include <pcl/point_types.h>

namespace Gripper{

  double Shape::intersectionVolumeHeuristic(const Shape& s) const{
    throw std::logic_error("Asking for intersection heuristic to a shape which doesn't know any!");
    return 0;
  }

  double Shape::haveNoIntersection(const Shape& s) const{
    throw std::logic_error("Asking for noIntersection heuristic to a shape which doesn't know any!");
    return 0;
  }
 
  Shape::ShapeList Shape::intersectionHeuristic() const {
    return {};
  }

  Shape::ShapeList Shape::noIntersectionHeuristic() const {
    return {};
  }


  std::string Shape::getID() const {
    return "Generic";
  }

  double Shape::getIntersectionVolume(const Shape& s) const {
    if(s.getID() == "Composed"){
      return s.getIntersectionVolume(*this);
    }

    if(haveNoIntersectionHeuristic(s) && haveNoIntersection(s)){
      return 0;
    }
    if(s.haveNoIntersectionHeuristic(*this) && s.haveNoIntersection(*this)){
      return 0;
    }

    if(haveIntersectionHeuristic(s)){
      return intersectionVolumeHeuristic(s);
    }
    if(s.haveIntersectionHeuristic(*this)){
      return s.intersectionVolumeHeuristic(*this);
    }
    
    const Shape& smaller=(getVolume() > s.getVolume() ? *this : s);
    const Shape& bigger=(getVolume() > s.getVolume() ? s : *this);

    std::array<size_t, 3> toTry={2,10,30};
    for(size_t surfLevel : toTry){
      auto surfApprox=smaller.getCubettiSurface(surfLevel);
      size_t countSurf=bigger.countContainedPoints(surfApprox);
      if(countSurf){
        if(countSurf==surfApprox.cols()){
          return smaller.getVolume();
        }
        else{
          auto cubes=smaller.getCubettiVolume(BASE_APPROX_LEVEL);
          return (bigger.countContainedPoints(cubes)*smaller.getVolume())/cubes.cols();
        }
      }
    }
    return 0;
  }

  const std::vector<double>& Shape::getDimensions() const {
    return _dimensions;
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


  Shape::~Shape(){
  }

  Shape::Ptr operator*(const Eigen::Affine3d& lhs, const Shape::Ptr& rhs){
    std::shared_ptr<Shape> result{rhs->clone()};
    result->_pose=lhs*rhs->_pose;
    return Shape::Ptr{result};
  }

  bool Shape::haveIntersectionHeuristic(const Shape& s) const {
    for(const auto& x: intersectionHeuristic()){
      if(x==s.getID() || x=="Anything"){
        return true;
      }
    }
    return false;
  }
  
  bool Shape::haveNoIntersectionHeuristic(const Shape& s) const {
    for(const auto& x: noIntersectionHeuristic()){
      if(x==s.getID() || x=="Anything"){
        return true;
      }
    }
    return false;
  }

  void Shape::writeTo(cv::FileStorage& fs) const{
    fs << "{";
    fs << "name" << getID();
    fs << "dimensions" << getDimensions();
    fs << "pose" << getPose().matrix();
    fs << "}";
  }

}
