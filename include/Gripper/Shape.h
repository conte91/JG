#pragma once
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>

namespace Gripper{
  /** Empty shape */
  class Shape {
    public:
      typedef Eigen::Affine3d RelPose;
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> Points;
      typedef Points::Ptr PointsPtr;
      typedef Eigen::Matrix<double, 4, Eigen::Dynamic> PointsMatrix;
      typedef std::shared_ptr<const Shape> Ptr;

      Shape(const RelPose& pose, const std::vector<double>& dims);

      virtual PointsMatrix getCubettiSurface(size_t level) const = 0;
      virtual PointsMatrix getCubettiVolume(size_t level) const = 0;
      virtual double getIntersectionVolume(const Shape& s) const;
      virtual RelPose getPose() const final;
      virtual const std::vector<double>& getDimensions() const final;
      virtual PointsPtr getPCSurface(size_t level) const final;
      virtual PointsPtr getPCVolume(size_t level) const final;
      virtual std::string getID() const;
      virtual double getVolume() const = 0;
      virtual void writeTo(cv::FileStorage& fs) const;
      virtual Shape* clone() const = 0;

      virtual ~Shape();

    protected:
      typedef std::vector<std::string> KnownIntersections;
      typedef std::vector<std::string> ShapeList;
      std::vector<double> _dimensions;
      Eigen::Affine3d _pose;
      friend Shape::Ptr operator*(const Eigen::Affine3d& lhs, const Shape::Ptr& rhs);
      virtual size_t countContainedPoints(const PointsMatrix& pt) const ;

      virtual ShapeList intersectionHeuristic() const ;
      virtual ShapeList noIntersectionHeuristic() const ;

      virtual bool haveIntersectionHeuristic(const Shape& s) const final;
      virtual bool haveNoIntersectionHeuristic(const Shape& s) const final;

      virtual double intersectionVolumeHeuristic(const Shape& s) const;
      virtual double haveNoIntersection(const Shape& s) const;

      static constexpr size_t BASE_APPROX_LEVEL=100;
  };

  Shape::Ptr operator*(const Eigen::Affine3d& lhs, const Shape::Ptr& rhs);

}

namespace cv{
  void write( cv::FileStorage& fs, const std::string& name, const Gripper::Shape* const model);
  void read(const cv::FileNode& node, std::shared_ptr<const Gripper::Shape>& x, const std::shared_ptr<const Gripper::Shape>& default_value = std::shared_ptr<const Gripper::Shape>());
}
