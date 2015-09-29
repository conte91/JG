#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Camera/CameraModel.h>
#include <Recognition/Renderer3d.h>
#include <C5G/Pose.h>

int main(int argc, char** argv){
  if(argc!=9){
    std::cerr << "Usage: " << argv[0] << " path/to/object/mesh camera/model/file X Y Z a b g\n";
    return -1;
  }

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  const Camera::CameraModel& cam=Camera::CameraModel::readFrom(fs["camera_model"]);
  Renderer3d renderer(argv[1]);
  cv::Mat depth_out, image_out, mask_out;
  cv::Mat frame(480,640,CV_8UC3);
  frame.setTo(cv::Scalar{0,0,0});

  cv::Rect rect_out, rect_out2;
  renderer.set_parameters(cam, 0.1, 10.0);
  double x=atof(argv[3]);
  double y=atof(argv[4]);
  double z=atof(argv[5]);
  double a=atof(argv[6]);
  double b=atof(argv[7]);
  double g=atof(argv[8]);
  renderer.setObjectPose(C5G::Pose{x,y,z,a,b,g}.toTransform());
  renderer.renderDepthOnly(depth_out, mask_out, rect_out);
  renderer.renderImageOnly(image_out, rect_out);
  if(rect_out.width<0 || rect_out.height<0){
    std::cerr << "Object didn't render in a visible manner, sry\n";
    return -1;
  }
  cv::imshow("D" ,depth_out);
  cv::imshow("R" ,image_out);
  cv::imshow("M" ,mask_out);
  image_out.copyTo(frame(rect_out));
  cv::imshow("Final", frame);
  while((cv::waitKey() & 0xFF)!='q');
  return 0;
}
