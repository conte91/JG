#pragma once
#include <memory>
#include <string>
#include <Img/Image.h>

namespace Camera{
  class ImageProvider{
    protected:
      typedef Img::Image Image;
      const std::string _id;
    public:
      typedef std::shared_ptr<ImageProvider> Ptr;
      ImageProvider(const std::string& ID);
      virtual Image getFrame() const =0;
      virtual ~ImageProvider()=0;
  };
}
