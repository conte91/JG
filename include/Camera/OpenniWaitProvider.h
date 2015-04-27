#pragma once

#include <opencv2/opencv.hpp>
#include "OpenniProvider.h"

namespace Camera{
  class OpenNIWaitProvider : public OpenNIProvider {
    public:
      OpenNIWaitProvider(const std::string& ID="OpenNIWaitProvider");
      virtual Image getFrame() const;
  };
}
