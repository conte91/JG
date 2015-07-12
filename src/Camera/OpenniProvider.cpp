#include <Img/Image.h>
#include <Camera/OpenniProvider.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  OpenNIProvider::OpenNIProvider(const std::string& ID)
    :
      ImageProvider(ID),
      _capture(CV_CAP_OPENNI)
  {
  //  if(!_capture.isOpened()){
  //    _capture.open(cv::CV_CAP_OPENNI2);
      if(!_capture.isOpened()){
        throw std::string("couldn't open the capture device");
      }
    //}
  }

  Img::Image OpenNIProvider::getFrame() const {
    Image::Matrix depthMap, rgb;
    std::cout << "I'm taking a photo\n";
    _capture.grab();
    _capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP);
    _capture.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE);
    return Image(depthMap, rgb);
  }
}
