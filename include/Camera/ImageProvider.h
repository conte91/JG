#pragma once
#include <string>

namespace Camera{
  class ImageProvider{
    public:
      ImageProvider(const std::string& ID);
      virtual ~ImageProvider()=0;
  };
}
