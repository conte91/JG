#pragma once
#include <string>

namespace Camera{
  class ImageProvider{
    private:
      const std::string _id;
    public:
      ImageProvider(const std::string& ID);
      virtual ~ImageProvider()=0;
  };
}
