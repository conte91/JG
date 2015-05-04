#include <opencv2/opencv.hpp>
#include <unistd.h>
int main(int argc, char** argv){
  if(argc!=2){
    std::cout << "Nope.\n";
    return -1;
  }
  cv::VideoCapture _capture(CV_CAP_OPENNI);
  std::cout << "Started streaming data to main system..\n";
  int c=0;
  while(1){
    cv::Mat depthMap, rgb;
    usleep(500000);
    _capture.grab();
    _capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP);
    _capture.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE);
    imwrite(std::string(argv[1])+"/rgb.png", rgb);
    imwrite(std::string(argv[1])+"/depth.png", depthMap);
    std::cout << ++c << "\n";
  }
  return 0;
}
