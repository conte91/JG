#pragma once

#include "Shape.h"
namespace Gripper{
  class ComposedShape : public Shape {
    public:
      typedef std::vector<Shape> Components;
      ComposedShape(Eigen::Affine3d& pose, Components comp);
     
    protected:
      virtual std::string getID() const;
      virtual KnownIntersections getKnownIntersections() const;
      virtual double intersectionVolume(Shape& s) const;
      Components _components;
  };
};
