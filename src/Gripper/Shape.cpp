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
      return intersectionVolume(s);
    }
    assert(s.knowsHowToIntersect(*this) && "No valid intersections found between me and the other shape!!");
    return s.intersectionVolume(*this);
  }
  double Shape::intersectionVolume(const Shape& s) const {
    assert(false && "Empty shape cannot be intersect with anything.. This is probably a bug in the shape building process");
    return -1;
  }
  const std::vector<double>& Shape::getDimensions() const {
    return _dimensions;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr Shape::getPC() const
  {
    assert(false && "Empty shape cannot be transformed into point clouds.. This is probably a bug in the shape building process");
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
}
