#pragma once
#include <Img/Image.h>
#include <opencv2/core/operations.hpp>

namespace Camera{
  class CameraModel {
    public:
      /** Returns the 3x3 internal calibration matrix of the camera */
      cv::Mat getIntrinsic() const;

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
      CameraModel(int w, int h, double fx, double fy, double s, double xc, double yc);

      /** Build a camera model using a known camera matrix.
       * @param k Camera matrix for this model. Matrix MUST be a 3x3 float upper-triangular matrix, and focal lengths must be >0.
       */
      CameraModel(int w, int h, const cv::Mat& k);

      /** Returns the width of the frames */
      int getWidth() const;
      
      int getHeight() const;
      double getFx() const;
      double getFy() const;
      double getS() const;
      double getXc() const;
      double getYc() const;

    private:
      /** The intrinsic params of the camera */
      cv::Mat _K;

      /** Camera's frame size */
      int _imWidth;
      int _imHeight;

  };
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Camera::CameraModel& model);
  void read(const FileNode& node, Camera::CameraModel& x, const Camera::CameraModel& default_value = Camera::CameraModel(0,0,0,0,0,0,0));
}
