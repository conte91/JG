#pragma once
#include <opencv2/core/core.hpp>

namespace Img{
  struct Image{
      typedef cv::Mat Matrix;
      static const int ALLOWED_WIDTH;
      static const int ALLOWED_HEIGHT;
      Matrix depth;
      Matrix rgb;
      Image(const Matrix& d, const Matrix& r);
      Image();
  };
}
