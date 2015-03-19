#include <Camera/ImageProvider.h>

namespace Camera{
  ImageProvider::ImageProvider(const std::string& ID)
    :
      _id(ID)
  {
  };
  ImageProvider::~ImageProvider(){};
}
