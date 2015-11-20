#pragma once

#include "ImageProvider.h"

namespace Camera{
  class FileProviderAuto : public ImageProvider {
    public:
    FileProviderAuto( const std::string& rgbFile, const std::string& depthFile, const std::string& ID="FileProviderAuto");
      virtual Img::Image getFrame() const;
      Img::Image::Matrix _rgb, _depthMap;
  };
}
