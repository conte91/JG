#include <Img/Image.h>
#include <stdexcept>
#include <Camera/Openni2Provider.h>
#include <opencv2/opencv.hpp>


namespace Camera{
  OpenNI2Provider::OpenNI2Provider(const std::string& ID)
    :
      ImageProvider(ID)
  {
      _capture.open();
      if(!_capture.isOpen()){
        throw std::runtime_error("couldn't open the capture device");
      }
  }

  Img::Image OpenNI2Provider::getFrame() const {
    Image::Matrix depthMap, rgb;
    std::cout << "I'm taking a photo\n";
    _capture.grab();
    _capture.retrieve(rgb, depthMap);
    return Image(depthMap, rgb);
  }
}
