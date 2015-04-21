#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include "Image.h"

namespace Camera{
  class ImageProvider{
    private:
      const std::string _id;
    public:
      typedef boost::shared_ptr<ImageProvider> Ptr;
      ImageProvider(const std::string& ID);
      virtual Image getFrame() const=0;
      virtual ~ImageProvider()=0;
  };
}
