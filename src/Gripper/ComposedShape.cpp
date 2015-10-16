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
      result+=x.getIntersectionVolume(s);
    }
    return result;
  }
  ComposedShape::ComposedShape(Eigen::Affine3d& pose, Components comp)
    :
      Shape(pose, {}),
      _components(comp)
  {
  }
}