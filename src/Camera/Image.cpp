#include <Camera/Image.h>

namespace Camera{
  Image::Image(const Matrix& d, const Matrix& r)
    :
      depth(d),
      rgb(r)
  {

  }
  Image::Image(){
  }

}
