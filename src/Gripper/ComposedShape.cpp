#include <Gripper/ComposedShape.h>

namespace Gripper{
  std::string ComposedShape::getID() const {
    return "Composed";
  }
  auto ComposedShape::getKnownIntersections() const -> KnownIntersections{
    return {"Anything"};
  }

  double ComposedShape::intersectionVolume(Shape& s) const {
    double result=0;
    for(const auto& x : _components){
      result+=x->getIntersectionVolume(s);
    }
    return result;
  }
  ComposedShape::ComposedShape(const Eigen::Affine3d& pose, const Components& comp)
    :
      Shape(pose, {}),
      _components(comp)
  {
  }

  ComposedShape::PointsMatrix ComposedShape::getCubettiSurface(size_t level) const {
    PointsMatrix result(4,0);
    for(const auto& x: _components){
      const auto& partial=x->getCubettiSurface(level);
      size_t curSize=result.cols();
      result.conservativeResize(Eigen::NoChange,curSize+partial.cols());
      result.rightCols(partial.cols())=partial;
    }
    return _pose*result;
  }
  ComposedShape::PointsMatrix ComposedShape::getCubettiVolume(size_t level) const{
    PointsMatrix result(4,0);
    for(const auto& x: _components){
      const auto& partial=x->getCubettiVolume(level);
      size_t curSize=result.cols();
      result.conservativeResize(Eigen::NoChange,curSize+partial.cols());
      result.rightCols(partial.cols())=partial;
    }
    return _pose*result;
  }

  ComposedShape::~ComposedShape(){
  }
}
