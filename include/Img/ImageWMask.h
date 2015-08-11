#pragma once
#include "Image.h"

namespace Img{
  struct ImageWMask : public Image {
    Matrix mask;
    ImageWMask (const Matrix& d, const Matrix& r, const Matrix& m);
    ImageWMask (const Image& src, const Matrix& m);
    ImageWMask();
  };
}

