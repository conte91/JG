#pragma once

#include "Shape.h"
namespace Gripper{
  class Sphere : public Shape {
    public:
      Sphere(const RelPose& pose, double R);
      virtual std::string getID() const override;
    protected:
      virtual KnownIntersections getKnownIntersections() const override;
      virtual double intersectionVolume(const Shape& s, size_t level) const override;
      virtual PointsMatrix getCubettiSurface(size_t level) const override;
      virtual Eigen::Matrix<double, 4, Eigen::Dynamic>  getCubettiVolume(size_t level) const override;
  };
};
