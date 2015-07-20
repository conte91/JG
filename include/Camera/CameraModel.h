#pragma once
#include <Img/Image.h>

namespace Camera{
  class CameraModel {
    public:
      /** Returns the 3x3 internal calibration matrix of the camera */
      cv::Mat getIntrinsic() const;

      /** Reads a model from a JSON file.
       * @param filename the name of the file to read from 
       */
      static CameraModel fromFile(const std::string& filename);

      /** Writes a model to a JSON file.
       * @param filename the name of the file to save to
       */
      void toFile(const std::string& filename) const;

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
