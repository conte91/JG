#include <Img/Image.h>
#include <Camera/Openni1Provider.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  OpenNI1Provider::OpenNI1Provider(const std::string& ID)
    :
      ImageProvider(ID),
      _capture(CV_CAP_OPENNI)
  {
  //  if(!_capture.isOpened()){
  //    _capture.open(cv::CV_CAP_OPENNI2);
      if(!_capture.isOpened()){
        throw std::string("couldn't open the capture device");
      }
      if(!_capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) ){
        std::cout << "Setting OPENNI registration flag,,\n";
        _capture.set(CV_CAP_PROP_OPENNI_REGISTRATION,1);
        if(!_capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) ){
          throw std::string("couldn't set registration flag");
        }
      }

    //}
  }

  Img::Image OpenNI1Provider::getFrame() const {
    Image::Matrix depthMap, rgb;
    std::cout << "I'm taking a photo\n";
    _capture.grab();
    _capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP);
    _capture.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE);
    return Image(depthMap, rgb);
  }
}
