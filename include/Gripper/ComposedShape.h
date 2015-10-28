#pragma once

#include <pcl/point_types.h>
#include "Shape.h"
namespace Gripper{
  class ComposedShape : public Shape {
    public:
      typedef std::vector<Shape> Components;
      ComposedShape(Eigen::Affine3d& pose, Components comp);
      virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPC() const;
     
    protected:
      virtual std::string getID() const;
      virtual KnownIntersections getKnownIntersections() const;
      virtual double intersectionVolume(Shape& s) const;
      
      Components _components;
  };
};
