#pragma once

#include "ImageProvider.h"

namespace Camera{
  class FileProvider : public ImageProvider {
    public:
      FileProvider(const std::string& ID="FileProvider");
      virtual Image getFrame() const;
  };
}
