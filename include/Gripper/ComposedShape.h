#pragma once

#include <pcl/point_types.h>
#include "Shape.h"
namespace Gripper{
  class ComposedShape : public Shape {
    public:
      typedef std::vector<std::shared_ptr<Shape> > Components;
      ComposedShape(const Eigen::Affine3d& pose, const Components& comp);
      virtual PointsMatrix getCubettiVolume(size_t level) const override;
      virtual PointsMatrix getCubettiSurface(size_t level) const override;
     
      virtual ~ComposedShape();
    protected:
      virtual std::string getID() const;
      virtual KnownIntersections getKnownIntersections() const;
      virtual double intersectionVolume(Shape& s) const;
      
      Components _components;
  };
};
