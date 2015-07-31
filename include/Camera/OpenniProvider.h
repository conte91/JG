#pragma once

#include <opencv2/opencv.hpp>
#include "ImageProvider.h"

namespace Camera{

  /** Generic OpenNI wrapper, which includes OpenNI version 1 and 2 */
  class OpenNIProvider : public ImageProvider {
    public:
      OpenNIProvider(const std::string& ID="OpenniProvider", int preferredVersion=1);
      virtual Image getFrame() const;

    private:
      std::unique_ptr<ImageProvider> _p;

  };
}
