#include <Img/Image.h>
#include <Camera/FileProviderAuto.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  FileProviderAuto::FileProviderAuto(const std::string& rgbFile, const std::string& depthFile, const std::string& ID)
  :
  ImageProvider(ID)
  {
        _rgb=cv::imread(rgbFile, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
        _depthMap=cv::imread(depthFile, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_GRAYSCALE);
  }

  Img::Image FileProviderAuto::getFrame() const {
    return Image(_depthMap, _rgb);
  }
}
