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

      virtual Shape* clone() const override;

    protected:
      virtual size_t countContainedPoints(const PointsMatrix& pt) const override;
  };
}
