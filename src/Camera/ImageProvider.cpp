#include <Camera/ImageProvider.h>
#include <Img/Image.h>

namespace Camera{
  ImageProvider::ImageProvider(const std::string& ID)
    :
      _id(ID)
  {
  };

  ImageProvider::~ImageProvider(){};
}
