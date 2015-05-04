#include <Camera/OpenniStreamProvider.h>
#include <Camera/Image.h>

namespace Camera{
  OpenniStreamProvider::OpenniStreamProvider(const std::string& path, const std::string& ID)
    :
	ImageProvider(ID),
  _path(path)
  {
    std::cout << ImageProvider::_id << ": set default image path to " << _path+"/rgb.png" << " and "<< _path+"/depth.png" << "\n";
  };
	
  Image OpenniStreamProvider::getFrame() const {
    std::cout << "Waiting for a frame to arrive..\n";
    system("touch /tmp/shameonme");
    std::cout << "Lock acquired. Waiting for release..\n";
    while(!system("ls /tmp/shameonme > /dev/null"));
    std::cout << "Done.\n";
    Image::Matrix rgb=cv::imread(_path+"/rgb.png");
    Image::Matrix depthMap=cv::imread(_path+"/depth.png");
    return Image(depthMap, rgb);
	}
  OpenniStreamProvider::~OpenniStreamProvider(){};
}

