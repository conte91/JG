#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <Camera/ImageProvider.h>
#include <Camera/OpenniProvider.h>
#include <Camera/DummyProvider.h>
#include <Camera/FileProvider.h>
#include <Camera/OpenniWaitProvider.h>
#include <Camera/CameraModel.h>
#include <Recognition/Mesh.h>
#include <Recognition/Renderer3d.h>
#include <C5G/Pose.h>

int main(int argc, char** argv){
  using Camera::ImageProvider;

  if(argc!=9){
    std::cerr << "Usage: " << argv[0] << " path/to/object/mesh camera/model/file X Y Z a b g\n";
    return -1;
  }
  std::cout << "Initializing camera and meshes..\n";

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  auto camModel=Camera::CameraModel::readFrom(fs["camera_model"]);
  Mesh mesh(argv[1]);
  std::cout << "Done.\n";

  auto& renderer=Recognition::Renderer3d::globalRenderer();
  renderer.set_parameters(camModel, 0.1, 10.0, "Renderer test");
  double x=atof(argv[3]);
  double y=atof(argv[4]);
  double z=atof(argv[5]);
  double a=atof(argv[6]);
  double b=atof(argv[7]);
  double g=atof(argv[8]);
  cv::Mat depth_out, image_out, mask_out;
  cv::Rect rect_out, rect_out2;
  renderer.setObjectPose(C5G::Pose{x,y,z,a,b,g}.toTransform());
  renderer.renderDepthOnly(mesh,depth_out, mask_out, rect_out);
  renderer.renderImageOnly(mesh,image_out, rect_out);
  cv::Mat nznz;
  depth_out.convertTo(nznz, CV_32FC1, 1e-03);
  cv::Mat resss(camModel.getWidth(), camModel.getHeight(), CV_8UC3);
  cv::Mat resssD(camModel.getWidth(), camModel.getHeight(), CV_32FC1);
  cv::Mat resssM(camModel.getWidth(), camModel.getHeight(), CV_8UC1);
  resss.setTo(cv::Scalar{0,0,0});
  resssD.setTo(cv::Scalar{0});
  resssM.setTo(cv::Scalar{0});
  nznz.copyTo(resssD(rect_out));
  image_out.copyTo(resss(rect_out));
  mask_out.copyTo(resssM(rect_out));

  cv::imshow("R" ,image_out);
  cv::Mat do2;
  depth_out.convertTo(do2, CV_32FC1, 1e-3);
  auto points=camModel.sceneToCameraPointCloud(resss, resssD,resssM);
  pcl::visualization::PCLVisualizer viewer("Scene's PCL");
  viewer.addCoordinateSystem(1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colors (points);
  viewer.addPointCloud<pcl::PointXYZRGB>(points, colors, "scene");
  cv::imshow("Scene", image_out);
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
    cv::waitKey(100);
  }
}

