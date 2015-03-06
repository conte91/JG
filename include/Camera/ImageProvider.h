#pragma once
#include <string>

namespace Camera{
  class ImageProvider{
    /*private:
      virtual ImageProvider()=0;*/
    public:
      ImageProvider(const std::string& ID);
      virtual ~ImageProvider()=0;
  };
}
