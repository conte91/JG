#pragma once
#include <opencv2/core/core.hpp>

namespace Camera{
  struct Image{
      typedef cv::Mat Matrix;
      Matrix depth;
      Matrix rgb;
      Image(const Matrix& d, const Matrix& r);
      Image();
  };
}
