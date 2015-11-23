#pragma once

#include "Shape.h"
namespace Gripper{
  class Sphere : public Shape {
    public:
      Sphere(const RelPose& pose, double R);
      virtual std::string getID() const override;
    protected:
      virtual double haveNoIntersection(const Shape& s) const override;
      virtual double intersectionVolumeHeuristic(const Shape& s) const override;
      virtual PointsMatrix getCubettiSurface(size_t level) const override;
      virtual Eigen::Matrix<double, 4, Eigen::Dynamic>  getCubettiVolume(size_t level) const override;
      virtual double getVolume() const override;

      virtual ShapeList intersectionHeuristic() const override;
      virtual ShapeList noIntersectionHeuristic() const override;

      virtual size_t countContainedPoints(const Shape::PointsMatrix& pt) const override;
  };
};
