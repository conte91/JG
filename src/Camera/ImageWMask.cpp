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
    CV_Assert(m.depth()==CV_8U && m.channels()==1);
    CV_Assert(m.rows==ALLOWED_HEIGHT && m.cols==ALLOWED_WIDTH);
  }

  ImageWMask::ImageWMask()
    :
      Image()
  {
  }
}
