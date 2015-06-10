#include <Camera/Image.h>

namespace Camera{

  const int Image::ALLOWED_WIDTH=640;
  const int Image::ALLOWED_HEIGHT=480;

  Image::Image(const Matrix& d, const Matrix& r)
    :
      depth(d),
      rgb(r)
  {
    CV_Assert(d.rows==ALLOWED_HEIGHT && d.cols==ALLOWED_WIDTH);
    CV_Assert(r.rows==ALLOWED_HEIGHT && r.cols==ALLOWED_WIDTH);
    CV_Assert(r.depth()==CV_8U && r.channels() == 3);
    CV_Assert(d.depth()==CV_16U && d.channels() == 1);
  }

  Image::Image(){
  }

}
