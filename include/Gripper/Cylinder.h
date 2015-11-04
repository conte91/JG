#pragma once
#include <Eigen/Geometry>
#include "Shape.h"

namespace Gripper{
  /** Cylinder of base radius R and heigth H */
  class Cylinder : public Shape {
    public:
      Cylinder(Eigen::Affine3d pose, double R, double H);
      virtual ~Cylinder();

      virtual PointsMatrix getCubettiSurface(size_t level) const override;
      virtual PointsMatrix getCubettiVolume(size_t level) const override;
      virtual std::string getID() const override;
      virtual double getVolume() const override;

    protected:
      virtual double intersectionVolume(const Shape& s, size_t level) const override;
      virtual KnownIntersections getKnownIntersections() const override;
      friend Shape operator*(const Eigen::Affine3d& lhs, const Shape& rhs);
      virtual size_t countContainedPoints(const PointsMatrix& pt) const override;
  };
}
