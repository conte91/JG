#include <Camera/CameraView.h>
#include <highgui.h>

namespace Camera{

//SUGGESTION: compile me
  CameraView::CameraView(std::shared_ptr<const ImageProvider> prov, const std::string& ID)
  : ImageConsumer(prov, ID), _ID(ID){
    cv::namedWindow(_ID+"::depth");
    cv::namedWindow(_ID+"::rgb");
  }

  void CameraView::Update(){
    cv::imshow(_ID+"::depth", cv::Mat(1, 1, CV_8UC1));
  }

  CameraView::~CameraView(){
    cv::destroyWindow(_ID+"::depth");
    cv::destroyWindow(_ID+"::rgb");
  }

}
