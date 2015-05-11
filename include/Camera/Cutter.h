#include <Camera/Image.h>

namespace Camera{
  Image cutImage(const Image& input, int x0, int y0, int x1, int y1);
  Image::Matrix getMask(const Image& input, int x0, int y0, int x1, int y1);
}
