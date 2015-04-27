#include <Camera/Image.h>
#include <Camera/OpenniWaitProvider.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  OpenNIWaitProvider::OpenNIWaitProvider(const std::string& ID)
    :
      OpenNIProvider(ID)
  {
  }

  Image OpenNIWaitProvider::getFrame() const {
    std::cout << 
      "==============================================="
      "||   Position the camera and press Enter..   ||"
      "==============================================="
      "\n";
    return OpenNIProvider::getFrame();
  }
}
