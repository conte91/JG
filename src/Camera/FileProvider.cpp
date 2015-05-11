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
    auto f = [] (std::string query) -> Image::Matrix {
      bool haveFinished=false;
      Image::Matrix rgb;
      while(!haveFinished){
        std::cout << query;
        std::string s;
        std::cin >> s;
        rgb=cv::imread(s);
        haveFinished=true;
        if(rgb.empty()){
          std::cout << "Try again.\n";
          haveFinished=false;
        }
      }
      return rgb;
    };

    Image::Matrix depthMap=f("Depth path:"), rgb=f("RGB path:");
    return Image(depthMap, rgb);
  }
}
