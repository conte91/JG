#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include <Img/Image.h>

namespace Camera{
  class ImageProvider{
    protected:
      typedef Img::Image Image;
      const std::string _id;
    public:
      typedef boost::shared_ptr<ImageProvider> Ptr;
      ImageProvider(const std::string& ID);
      virtual Image getFrame() const =0;
      virtual ~ImageProvider()=0;
  };
}
