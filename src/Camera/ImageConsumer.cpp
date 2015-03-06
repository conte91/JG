#include <Camera/ImageConsumer.h>

namespace Camera{
  ImageConsumer::~ImageConsumer(){

  }

  ImageConsumer::ImageConsumer(boost::shared_ptr<const ImageProvider> provider, const std::string& ID)
    :
      _provider(provider)
  {
  }

  void ImageConsumer::setProvider(boost::shared_ptr<const ImageProvider> provider){
    _provider=provider;
  }
}

