#pragma once

#include "Shape.h"
namespace Gripper{
  class Sphere : public Shape {
    public:
      Sphere(const RelPose& pose, double R);
    protected:
      virtual std::string getID() const override;
      virtual KnownIntersections getKnownIntersections() const override;
      virtual double intersectionVolume(const Shape& s) const override;
  };
};
