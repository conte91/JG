#include <Camera/Image.h>
#include <Camera/IndexTaker.h>

namespace Camera{
  Image cutImage(const Image& input, int x0, int y0, int x1, int y1);
  Image::Matrix getMask(const Image& input, const IndexTaker& taker);
  Image::Matrix getMask(const Image& input, int x0, int y0, int x1, int y1);
}
