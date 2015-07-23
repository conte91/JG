#include <Camera/OpenniWaitProvider.h>
#include <Camera/CameraModel.h>
#include <opencv2/rgbd.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

std::string type2str(int type) {
  using std::string;
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

cv::Mat r,d;
std::shared_ptr<Camera::CameraModel> cam;

void doSomething(int event, int x, int y, int flags, void* stafava){
  if(event==CV_EVENT_LBUTTONDOWN){
    std::cout << "Click @" << x << "," << y << "\n";
    cv::Vec3b rgb=r.at<cv::Vec3b>(y,x);
    unsigned int r=rgb.val[2], g=rgb.val[1], b=rgb.val[0];
    std::cout << "RGB: (" << r << "," << g << "," <<  b << ")\n - Coordinates: ";
    cv::Mat_<cv::Vec3f> depthOut;

    cv::Mat singlePointMask(d.rows, d.cols, CV_8U);
    singlePointMask.setTo(0);
    singlePointMask.at<uint8_t>(y,x)=255;

    cv::rgbd::depthTo3d(d, cam->getIntrinsic(), depthOut, singlePointMask);
    std::cout << "(" << depthOut[0][0].val[0];
    std::cout << "," << depthOut[0][0].val[1];
    std::cout << "," << depthOut[0][0].val[2] << ")\n";

  }
}

int main(){
  using Camera::OpenNIWaitProvider;
  using Camera::CameraModel;

  OpenNIWaitProvider p;
  cam=std::shared_ptr<CameraModel>(new CameraModel(std::move(CameraModel::fromFile("camera_model.data"))));

  Img::Image i=p.getFrame();

  r=i.rgb;
  d=i.depth;

  std::cout << "Frame taken.\nRGB of type " << type2str(r.type()) << " and size " << r.rows << "x" << r.cols << ".\nDepth of type: " << type2str(d.type())  << "and size " << r.rows << "x" << r.cols << ".\n";

  cv::namedWindow("RGB image");
  cv::namedWindow("Depth image");
  cv::imshow("RGB image", r);
  cv::imshow("Depth image", d);

  cv::setMouseCallback("RGB image", doSomething);
  cv::setMouseCallback("Depth image", doSomething);

  while(cv::waitKey(0)!='q');
  return 0;

}
