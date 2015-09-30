#pragma once
#include <Img/Image.h>
#include <Img/ImageWMask.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/operations.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Camera{
  class CameraModel {
    public:
      /** Returns the 3x3 internal calibration matrix of the camera */
      cv::Matx33f getIntrinsic() const;

      /** Reads a model from a YAML file.
       * @param filename the name of the file to read from 
       */
      static CameraModel readFrom(const cv::FileNode& fs);

      /** Writes a model to a YAML file.
       * @param filename the name of the file to save to
       */
      void writeTo(const std::string& name, cv::FileStorage& fs) const;

      /** Build a camera model with the geometric parametrs 
       * @param fx Focal length (X)
       * @param fy Focal length (Y)
       * @param s Axis skew 
       * @param xc Principal point offset (X)
       * @param yc Principal point offset (Y)
       */
      CameraModel(int w, int h, float fx, float fy, float s, float xc, float yc, float xCam, float yCam, float zCam, float aCam, float bCam, float gCam);

      CameraModel(int w, int h, float fx, float fy, float s, float xc, float yc, const cv::Mat& extr_in = cv::Mat::eye(4,4, CV_32F));

      /** Build a camera model using a known camera matrix.
       * @param k Camera matrix for this model. Matrix MUST be a 3x3 float upper-triangular matrix, and focal lengths must be >0.
       */
      CameraModel(int w, int h, const cv::Matx33f& k, const cv::Matx44f& extr_in = cv::Matx44f::eye());

      /** Returns the width of the frames */
      int getWidth() const;
      
      int getHeight() const;
      float getFx() const;
      float getFy() const;
      float getS() const;
      float getXc() const;
      float getYc() const;

      Eigen::Affine3f getExtrinsic() const ;

      Eigen::Vector3d uvzToCameraFrame(double u, double v, double d) const;
      Eigen::Vector3d uvzToWorldFrame(double u, double v, double d) const;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneToCameraPointCloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Mat& mask=cv::Mat()) const ;
    private:
      /** The intrinsic params of the camera */
      cv::Matx33f _K;
      cv::Matx44f _extr;

      /** Camera's frame size */
      int _imWidth;
      int _imHeight;

  };
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Camera::CameraModel& model);
  void read(const FileNode& node, Camera::CameraModel& x, const Camera::CameraModel& default_value = Camera::CameraModel(0,0,0,0,0,0,0));
}
