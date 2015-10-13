#pragma once

#include "Shape.h"
namespace Gripper{
  class Cuboid : public Shape {
    public:
      Cuboid(Eigen::Affine3d pose, double W, double H, double D);
    protected:
      virtual std::string getID() const override;
      virtual KnownIntersections getKnownIntersections() const override;
      virtual double intersectionVolume(const Shape& s) const override;
    private:
      double W() const ;
      double H() const ;
      double D() const ;
      static constexpr size_t NCUT=20;
      static Eigen::Matrix<double, 4, Eigen::Dynamic> getCubettiMatrix(std::vector<double> dimensions);
  };
}
