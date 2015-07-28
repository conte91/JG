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

      /** 
       * @param fx Focal length (X)
       * @param fy Focal length (Y)
       * @param s Axis skew 
       * @param xc Principal point offset (X)
       * @param yc Principal point offset (Y)
       */
      CameraModel(float fx, float fy, float s, float xc, float yc);

    private:
      /** The intrinsic params of the camera */
      cv::Mat _K;

  };
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Camera::CameraModel& model);
  void read(const FileNode& node, Camera::CameraModel& x, const Camera::CameraModel& default_value = Camera::CameraModel(0,0,0,0,0));
}
