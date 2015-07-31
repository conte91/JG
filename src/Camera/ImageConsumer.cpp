#include <Camera/ImageConsumer.h>

namespace Camera{
  ImageConsumer::~ImageConsumer(){

  }

  ImageConsumer::ImageConsumer(std::shared_ptr<const ImageProvider> provider, const std::string& ID)
    :
      _provider(provider)
  {
  }

  void ImageConsumer::setProvider(std::shared_ptr<const ImageProvider> provider){
    _provider=provider;
  }
}

