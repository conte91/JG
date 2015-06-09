#include <algorithm>
#include <cstdlib>
#include <Camera/Cutter.h>
#include <Camera/Image.h>
#include <Camera/IndexTaker.h>
#include <cv.h>

namespace Camera{
  Image cutImage(const Image& input, int x0, int y0, int x1, int y1){
    cv::Mat depth, rgb;
    input.depth(cv::Rect(x0, y0, (x1-x0), (y1-y0))).copyTo(depth);
    input.rgb(cv::Rect(x0, y0, (x1-x0), (y1-y0))).copyTo(rgb);
    return Image(depth, rgb);
  }

  Image::Matrix getMask(const Image& input, const IndexTaker& taker){
    return getMask(input, taker.getX1(), taker.getY1(), taker.getX2(), taker.getY2());
  }

  Image::Matrix getMask(const Image& input, int x0, int y0, int x1, int y1){
    using std::min;
    using std::max;
    Image::Matrix theMask(cv::Mat::zeros(input.rgb.rows, input.rgb.cols, CV_8U));
    theMask(cv::Rect(min(x0, x1), min(y0, y1),abs(x0-x1),abs(y0-y1))).setTo(255);
    return theMask;
  }
}
