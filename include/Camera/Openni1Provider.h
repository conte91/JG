#pragma once

#include <opencv2/opencv.hpp>
#include "ImageProvider.h"

namespace Camera{
  class OpenNI1Provider : public ImageProvider {
    public:
      OpenNI1Provider(const std::string& ID="OpenniStreamProvider");
      virtual Image getFrame() const;

    private:
      mutable cv::VideoCapture _capture;


  };
}
