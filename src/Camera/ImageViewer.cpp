#include "Image.h"
#include <cv/highgui.h>

namespace Camera{
  ImageViewer::ImageViewer(const std::string& ID)
    :
      _ID(ID)
  {
      cv::namedWindow(_ID+" (RGB)", cv::WINDOW_AUTOSIZE);
      cv::namedWindow(_ID+" (DEPTH)", cv::WINDOW_AUTOSIZE);
  }
  void ImageViewer::updateMyself(){
    cv::destroyAllWindows(0);
    cv::imshow(_ID+" (RGB)", _what.rgb);
    cv::imshow(_ID+" (DEPTH)", _what.depth);
    cv::putText(_ID+" (RGB)", _title+" RGB");
    cv::putText(_ID+" (DEPTH)", _title+" DEPTH");
    cv::waitKey(0);
  }

  void showImage(const Image& what){
    _what=what;
    updateMyself();
  }

  void setTitle(const std::string& title){
    _title=title;
    updateMyself();
  }

}
