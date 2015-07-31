#pragma once

#include <opencv2/opencv.hpp>
#include "ImageProvider.h"
#include "cvni2.h"

namespace Camera{
  class OpenNI2Provider : public ImageProvider {
    public:
      OpenNI2Provider(const std::string& ID="OpenniStreamProvider");
      virtual Image getFrame() const;

    private:
      mutable CvNI2 _capture;

  };
}
