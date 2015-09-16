#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Camera/CameraModel.h>
#include <Recognition/Renderer3d.h>

int main(int argc, char** argv){
  if(argc!=9){
    std::cerr << "Usage: " << argv[0] << " path/to/object/mesh camera/model/file X Y Z a b g\n";
    return -1;
  }

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  const Camera::CameraModel& cam=Camera::CameraModel::readFrom(fs["camera_model"]);
  std::shared_ptr<Renderer3d> rendererPtr (new Renderer3d(argv[1]));
  cv::Mat depth_out, image_out, mask_out;
  cv::Rect rect_out, rect_out2;
  rendererPtr->set_parameters(cam.getWidth(), cam.getHeight(), cam.getFx(), cam.getFy(), 0.1, 1000.0);
  double x=atof(argv[3]);
  double y=atof(argv[4]);
  double z=atof(argv[5]);
  double a=atof(argv[6]);
  double b=atof(argv[7]);
  double g=atof(argv[8]);
  rendererPtr->lookAt(x,y,z,a,b,g);
  rendererPtr->renderDepthOnly(depth_out, mask_out, rect_out);
  rendererPtr->renderImageOnly(image_out, rect_out);
  cv::imshow("D" ,depth_out);
  cv::imshow("R" ,image_out);
  cv::imshow("M" ,mask_out);
  cv::waitKey();
  return 0;
}
