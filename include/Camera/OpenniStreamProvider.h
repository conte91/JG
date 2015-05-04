#pragma once

#include <opencv2/opencv.hpp>
#include "ImageProvider.h"

namespace Camera{
  class OpenniStreamProvider : public ImageProvider {
    public:
      OpenniStreamProvider(const std::string& path="./", const std::string& ID="OpenNIProvider");
      virtual Image getFrame() const;
      ~OpenniStreamProvider();

    private:
      std::string _path;


  };
}
