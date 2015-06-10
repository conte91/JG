#include <Camera/Image.h>
#include <Camera/FileProvider.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  FileProvider::FileProvider(const std::string& ID)
    :
      ImageProvider(ID)
  {
  }

  Image FileProvider::getFrame() const {
    auto f = [] (std::string query, int isColor) -> Image::Matrix {
      bool haveFinished=false;
      Image::Matrix rgb;
      while(!haveFinished){
        std::cout << query;
        std::string s;
        std::cin >> s;
        rgb=cv::imread(s, isColor);
        haveFinished=true;
        if(rgb.empty()){
          std::cout << "Try again.\n";
          haveFinished=false;
        }
      }
      return rgb;
    };

    Image::Matrix depthMap=f("Depth path:", CV_LOAD_IMAGE_ANYDEPTH|CV_LOAD_IMAGE_GRAYSCALE), rgb=f("RGB path:", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
    return Image(depthMap, rgb);
  }
}
