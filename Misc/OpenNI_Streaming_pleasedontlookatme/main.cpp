#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <unistd.h>
int main(int argc, char** argv){
  if(argc!=2){
    std::cout << "Nope.\n";
    return -1;
  }
  cv::VideoCapture _capture(CV_CAP_OPENNI);
  std::cout << "Started streaming data to main system..\n";
  int c=0;
  cv::namedWindow("DEPTH");
  while(1){
    cv::Mat depthMap, rgb;
    usleep(500000);
    _capture.grab();
    _capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP);
    cv::imshow("DEPTH", depthMap);
    cv::waitKey(1);
    _capture.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE);
    cv::imshow("RGB", rgb);
    cv::waitKey(1);
    std::string name=std::string(argv[1])+"/";
    imwrite(name+"/rgb1.png", rgb);
    imwrite(name+"/depth1.png", depthMap);
    system(("mv "+name+"rgb1.png "+name+"rgb.png ; mv "+name+"depth1.png "+name+"depth.png").c_str());
    std::cout << ++c << "\n";
  }
  return 0;
}
