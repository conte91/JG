#include <opencv2/opencv.hpp>
#include <Camera/DummyProvider.h>
#include <Camera/Image.h>

namespace Camera{

  DummyProvider::DummyProvider(const std::string& ID)
    :
      ImageProvider(ID){
  }

  Image DummyProvider::getFrame() const{
    std::cout << "I'm taking a photo\n";
    return Image(Image::Matrix(640, 480, cv::DataType<float>::type), Image::Matrix(640, 480, cv::DataType<float>::type));
  }

}
