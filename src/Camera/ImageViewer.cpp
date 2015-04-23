#include "Image.h"
#include <cv/highgui.h>

namespace Camera{
  ImageViewer::ImageViewer(const std::string& ID)
    :
      _ID(ID),
      _imgRGB(ID+" (RGB)"),
      _imgDEPTH(ID+" (DEPTH)")
  {
  }
  void ImageViewer::updateMyself(){
    cv::imshow(_ID+" (RGB)", _what.rgb);
    cv::imshow(_ID+" (DEPTH)", _what.depth);
    cv::putText(_ID+" (RGB)", _title+" RGB");
    cv::putText(_ID+" (DEPTH)", _title+" DEPTH");
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
