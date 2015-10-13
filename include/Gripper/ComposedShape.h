#pragma once

#include <pair>
#include "Shape.h"
namespace Gripper{
  class ComposedShape : public Shape {
    public:
      typedef std::pair<Eigen::Affine3d, Shape> Element;

    protected:
      virtual static std::string getID();
      virtual static KnownIntersections getKnownIntersections;
      virtual double intersectionVolume(Shape& s) const;
  };
};
