#pragma once

#include <opencv2/opencv.hpp>
#include "ImageProvider.h"

namespace Camera{
  class OpenNIProvider : public ImageProvider {
    public:
      OpenNIProvider(const std::string& ID="OpenniStreamProvider");
      virtual Image getFrame() const;

    private:
      mutable cv::VideoCapture _capture;


  };
}
