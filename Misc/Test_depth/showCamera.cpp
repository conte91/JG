#include <sstream>
#include <stdexcept>
#include <Camera/ImageProvider.h>
#include <Camera/OpenniProvider.h>
#include <Camera/DummyProvider.h>
#include <Camera/FileProvider.h>
#include <Camera/CameraModel.h>
#include <Camera/OpenniWaitProvider.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <unistd.h>
int main(int argc, char** argv){

  using Camera::ImageProvider;

  if(argc!=3){
    std::cout << "Nope. specify a dir!\n";
    return -1;
  }

  std::unique_ptr<ImageProvider> camera([&argv] () -> ImageProvider*  { 
    try{
      switch(argv[2][0]){
        case 'W':
          return new Camera::OpenNIWaitProvider();
        case 'O':
          return new Camera::OpenNIProvider();
        case 'F':
          return new Camera::FileProvider();
        case 'D':
          return new Camera::DummyProvider();
        default:
          throw std::runtime_error("No valid provider model specified");
      }
    }
    catch(std::string s){
      std::cerr << "Error initializing camera: " << s << "\n";
      exit(-1);
    }
  } ()
  );
  std::cout << "Started saving images..\n";
  cv::namedWindow("DEPTH");
  cv::namedWindow("RGB");

  int c=0;

  while(1){
    cv::Mat depthMap, rgb;
    usleep(500000);
    std::ostringstream rName, dName;
    rName << argv[1] << "/depth" << c << ".png";
    dName << argv[1] << "/rgb" << c << ".png";
    Img::Image i=camera->getFrame();
    cv::imshow("DEPTH", i.depth);
    cv::waitKey(1);
    cv::imshow("RGB", i.rgb);
    cv::waitKey(1);
    cv::imwrite(rName.str(), i.rgb);
    cv::imwrite(dName.str(), i.depth);
    c++;
  }
  return 0;
}
