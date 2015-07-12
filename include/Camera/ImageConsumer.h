#pragma once
#include <boost/shared_ptr.hpp>
#include "ImageProvider.h"
#include <Img/Image.h>

namespace Camera{
  /** Virtual class for a generic camera user. */
  class ImageConsumer {
    protected:
      typedef Img::Image Image;
      boost::shared_ptr<const ImageProvider> _provider;
    public:
      ImageConsumer(boost::shared_ptr<const ImageProvider> provider, const std::string& ID="");

      virtual ~ImageConsumer()=0;
      virtual void update()=0;
      virtual void setProvider(boost::shared_ptr<const ImageProvider> provider);
      Image getFrame();
  };

}
