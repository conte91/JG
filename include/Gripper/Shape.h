#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>

namespace Gripper{
  /** Empty shape */
  class Shape{
    public:
      typedef Eigen::Affine3d RelPose;
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> Points;
      typedef Points::Ptr PointsPtr;
      typedef Eigen::Matrix<double, 4, Eigen::Dynamic> PointsMatrix;

      Shape(const RelPose& pose, const std::vector<double>& dims);

      virtual PointsMatrix getCubettiSurface(size_t level) const;
      virtual PointsMatrix getCubettiVolume(size_t level) const;
      virtual double getIntersectionVolume(const Shape& s) const final;
      virtual RelPose getPose() const final;
      virtual const std::vector<double>& getDimensions() const final;
      virtual PointsPtr getPCSurface(size_t level) const final;
      virtual PointsPtr getPCVolume(size_t level) const final;
      virtual std::string getID() const;
      virtual double getVolume() const;
      virtual void writeTo(cv::FileStorage& fs) const;

      virtual ~Shape();

    protected:
      typedef std::vector<std::string> KnownIntersections;
      virtual double intersectionVolume(const Shape& s, size_t level) const;
      virtual KnownIntersections getKnownIntersections() const;
      virtual bool knowsHowToIntersect(const Shape& s) const final;
      std::vector<double> _dimensions;
      Eigen::Affine3d _pose;
      friend Shape operator*(const Eigen::Affine3d& lhs, const Shape& rhs);
      virtual size_t countContainedPoints(const PointsMatrix& pt) const;
      static constexpr size_t BASE_APPROX_LEVEL=100;
  };

  Shape operator*(const Eigen::Affine3d& lhs, const Shape& rhs);

}

namespace cv{
  void write( cv::FileStorage& fs, const std::string& name, const std::unique_ptr<Gripper::Shape>& model);
  void read(const cv::FileNode& node, std::unique_ptr<Gripper::Shape>& x, const std::unique_ptr<Gripper::Shape>& default_value = std::unique_ptr<Gripper::Shape>());
}
