#pragma once

#include "Shape.h"
namespace Gripper{
  class Cuboid : public Shape {
    public:
      Cuboid(Eigen::Affine3d pose, double W, double H, double D);
      virtual ~Cuboid();
      
    protected:
      virtual std::string getID() const override;
      virtual PointsMatrix getCubettiSurface(size_t level) const override;
      virtual PointsMatrix getCubettiVolume(size_t level) const override;
      virtual double getVolume() const override;
      virtual size_t countContainedPoints(const Cuboid::PointsMatrix& pt) const override;

    private:
      double W() const ;
      double H() const ;
      double D() const ;
  };
}
