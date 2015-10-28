#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

namespace Gripper{
  /** Empty shape */
  class Shape{
    public:
      typedef Eigen::Affine3d RelPose;

      Shape(const RelPose& pose, const std::vector<double>& dims);

      virtual double getIntersectionVolume(const Shape& s) const final;
      virtual RelPose getPose() const final;
      virtual const std::vector<double>& getDimensions() const final;
      virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPC() const;
      virtual std::string getID() const;


    protected:
      typedef std::vector<std::string> KnownIntersections;
      virtual double intersectionVolume(const Shape& s) const;
      virtual KnownIntersections getKnownIntersections() const;
      virtual bool knowsHowToIntersect(const Shape& s) const final;
      std::vector<double> _dimensions;
      Eigen::Affine3d _pose;
  };
}
