#include <iostream>
#include <Camera/Image.h>
#include <Camera/ImageViewer.h>
#include <highgui.h>

namespace Camera{
  ImageViewer::ImageViewer(const std::string& ID)
    :
      _ID(ID),
      _what()
  {
      cv::namedWindow(_ID+" (RGB)", cv::WINDOW_AUTOSIZE);
      cv::namedWindow(_ID+" (DEPTH)", cv::WINDOW_AUTOSIZE);
  }
  void ImageViewer::updateMyself(){
    cv::destroyAllWindows();
    cv::imshow(_ID+" (RGB)", _what.rgb);
    cv::imshow(_ID+" (DEPTH)", _what.depth);
    std::cout <<
      "**********************************************"
      << _title <<
      "**********************************************";

    //cv::putText(_ID+" (RGB)", _title+" RGB");
    //cv::putText(_ID+" (DEPTH)", _title+" DEPTH");
    cv::waitKey(0);
  }

  void ImageViewer::showImage(const Image& what){
    _what=what;
    updateMyself();
  }

  void ImageViewer::setTitle(const std::string& title){
    _title=title;
    updateMyself();
  }

}
