#include <Camera/ImageWMask.h>

namespace Camera{
  ImageWMask::ImageWMask(const Image& src, const Matrix& m)
  :
  Image(src.depth,src.rgb),
  mask(m)
  {
  }

  ImageWMask::ImageWMask(const Matrix& d, const Matrix& r, const Matrix& m)
  :
  Image(d,r),
  mask(m)
  {
  }

  ImageWMask::ImageWMask()
    :
      Image()
  {
  }
}
