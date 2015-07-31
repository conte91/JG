#pragma once
#include "ImageProvider.h"
#include <memory>
#include <Img/Image.h>

namespace Camera{
  /** Virtual class for a generic camera user. */
  class ImageConsumer {
    protected:
      typedef Img::Image Image;
      std::shared_ptr<const ImageProvider> _provider;
    public:
      ImageConsumer(std::shared_ptr<const ImageProvider> provider, const std::string& ID="");

      virtual ~ImageConsumer()=0;
      virtual void update()=0;
      virtual void setProvider(std::shared_ptr<const ImageProvider> provider);
      Image getFrame();
  };

}
