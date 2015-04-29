#include <Camera/OpenniStreamProvider.h>
#include <Camera/Image.h>

namespace Camera{
  OpenniStreamProvider::OpenniStreamProvider(const std::string& path, const std::string& ID)
    :
	ImageProvider(ID),
  _path(path)
  {
  };
	
  Image OpenniStreamProvider::getFrame() const {
    std::cout << "Waiting for a frame to arrive..\n";
    system("touch /tmp/shameonme");
    Image::Matrix rgb=cv::imread(_path+"/rgb.png", 0);
    Image::Matrix depthMap=cv::imread(_path+"/depth.png", 1);
    while(system("ls /tmp/shameonme > /dev/null"));
    return Image(depthMap, rgb);
	}
  OpenniStreamProvider::~OpenniStreamProvider(){};
}

