#include <pcl/point_types.h>
#include <stdexcept>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <Camera/ImageProvider.h>
#include <Camera/OpenniProvider.h>
#include <Camera/DummyProvider.h>
#include <Camera/FileProvider.h>
#include <Camera/OpenniWaitProvider.h>
#include <Camera/CameraModel.h>

int main(int argc, char** argv){
  using Camera::ImageProvider;

  if(argc<3){
    std::cerr << "Usage: " << argv[0] << "camera_type camera_model";
    return -1;
  }
  std::cout << "Initializing camera..\n";

  std::unique_ptr<ImageProvider> camera([&argv] () -> ImageProvider*  { 
    try{
      switch(argv[1][0]){
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
  std::cout << "Done.\n";

  cv::FileStorage cameraFile(argv[2], cv::FileStorage::READ);
  Img::Image x=camera->getFrame();

  auto camModel=Camera::CameraModel::readFrom(cameraFile["camera_model"]);

  auto points=camModel.sceneToCameraPointCloud(x.rgb, x.depth, cv::Mat());
  pcl::visualization::PCLVisualizer viewer("Scene's PCL");
  viewer.addCoordinateSystem(0.1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colors (points);
  viewer.addPointCloud<pcl::PointXYZRGB>(points, colors, "scene");
  cv::imshow("Scene", x.rgb);
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
    cv::waitKey(100);
  }
}

