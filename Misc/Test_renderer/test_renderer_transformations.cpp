#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Camera/CameraModel.h>
#include <Recognition/Renderer3d.h>
#include <Recognition/Utils.h>
#include <Recognition/Model.h>
#include <C5G/Pose.h>

int main(int argc, char** argv){
  if(argc!=9){
    std::cerr << "Usage: " << argv[0] << " path/to/object/mesh camera/model/file X Y Z a b g\n";
    return -1;
  }

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  const Camera::CameraModel& cam=Camera::CameraModel::readFrom(fs["camera_model"]);
  Mesh mesh(argv[1]);
  auto& renderer=Recognition::Renderer3d::globalRenderer();
  cv::Mat depth_out, image_out, mask_out;
  cv::Mat frame(480,640,CV_8UC3), mattia(480,640,CV_8UC3);
  frame.setTo(cv::Scalar{0,0,0});
  mattia.setTo(cv::Scalar{0,0,0});

  cv::Rect rect_out, rect_out2;
  renderer.set_parameters(cam, 0.1, 10.0);
  /*T*/
  double x=atof(argv[3]);
  double y=atof(argv[4]);
  double z=atof(argv[5]);
  /*Up*/
  double a=atof(argv[6]);
  double b=atof(argv[7]);
  double g=atof(argv[8]);
  auto openGLTrans=Recognition::tUpToOpenGLWorldTransform({x,y,z},{a,b,g});
  std::cout << "Up vector to world transform: \n"<< openGLTrans.matrix() << "\n\n";
  renderer.setObjectPose(openGLTrans);
  renderer.renderDepthOnly(mesh, depth_out, mask_out, rect_out);
  renderer.renderImageOnly(mesh, image_out, rect_out);
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

  Eigen::Affine3d transformation = Recognition::tUpToCameraWorldTransform({x,y,z},{a,b,g});
  Recognition::Model model("Mattia", std::string(argv[1]), cam);
  cv::Rect rect;
  cv::Mat image, depth, mask;
  /** Performs a render of the object at the desired camera's position and adds it to the trained templates */
  model.render(transformation, image, depth, mask, rect);
  cv::imshow("DO" ,depth);
  cv::imshow("RO" ,image);
  cv::imshow("MO" ,mask);
  image.copyTo(mattia(rect));
  cv::imshow("FinalO", frame);
  while((cv::waitKey() & 0xFF)!='q');
  return 0;
}
