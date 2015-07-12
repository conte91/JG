#include <iostream>
#include <Img/Image.h>
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
    _toRender=_what;
    ///cv::putText(_toRender.rgb, _title+" RGB", {0,0});
    ///cv::putText(_toRender.depth, _title+" DEPTH", {0,0});

    //cv::destroyAllWindows();
    cv::imshow(_ID+" (RGB)", _toRender.rgb);
    cv::imshow(_ID+" (DEPTH)", _toRender.depth);
    std::cout <<
      "\n**********************************************\n"
      << _title <<
      "\n**********************************************\n";

    //cv::putText(_ID+" (RGB)", _title+" RGB");
    //cv::putText(_ID+" (DEPTH)", _title+" DEPTH");
    cv::waitKey(1);
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
