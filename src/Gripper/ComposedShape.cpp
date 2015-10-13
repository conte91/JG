#include <Gripper/ComposedShape.h>

namespace Gripper{
  std::string ComposedShape::getID(){
    return "Composed";
  }
  auto ComposedShape::getKnownIntersections -> KnownIntersections{
    return {"Anything"};
  }

  virtual double intersectionVolume(Shape& s) const{
    static_assert(false);
  }
}
