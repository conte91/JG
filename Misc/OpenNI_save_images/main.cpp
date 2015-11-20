#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <unistd.h>
#include <Camera/CameraModel.h>
#include <Camera/OpenniProvider.h>

int main(int argc, char** argv){
  if(argc!=2){
    std::cout << "Usage: " << argv[0] << " output_folder\n";
    return -1;
  }
  std::cout << "Started saving frames into " << argv[1] << "..\n";
  cv::namedWindow("DEPTH");
  Camera::OpenNIProvider capture;
  for(size_t c=0; ; ++c){
    cv::Mat depthMap, rgb;
    usleep(500000);
    auto im=capture.getFrame(); 
    depthMap=im.depth;
    rgb=im.rgb;
    cv::imshow("DEPTH", depthMap);
    cv::imshow("RGB", rgb);
    cv::waitKey(1);
    std::ostringstream rgbName, depthName;

    rgbName << argv[1] << "/rgb" << c << ".png";
    depthName << argv[1] << "/depth" << c << ".png";
    cv::Mat ddd;
    depthMap.convertTo(ddd, CV_16UC1, 1000);
    imwrite(rgbName.str(), rgb);
    imwrite(depthName.str(), ddd);
    std::cout << "Saved images: " << c << "\n";
  }
  return 0;
}
