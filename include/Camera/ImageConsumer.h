#pragma once
#include <boost/shared_ptr.hpp>
#include "ImageProvider.h"

namespace Camera{
  /** Virtual class for a generic camera user. */
  class ImageConsumer {
    protected:
      boost::shared_ptr<const ImageProvider> _provider;
    public:
      ImageConsumer(boost::shared_ptr<const ImageProvider> provider, const std::string& ID="");

      virtual ~ImageConsumer()=0;
      virtual void Update()=0;
      virtual void setProvider(boost::shared_ptr<const ImageProvider> provider);
  };

}
