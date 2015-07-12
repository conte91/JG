#include <algorithm>
#include <cstdlib>
#include <cv.h>
#include <Img/Image.h>
#include <Img/Manipulation/Cutter.h>
#include <Img/Manipulation/IndexTaker.h>

namespace Img{
  namespace Manipulation{
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
      theMask.colRange(min(x0,x1), max(x0,x1)).rowRange(min(y0,y1),max(y0,y1)).setTo(255);
      return theMask;
    }
  }
}
