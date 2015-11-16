#include <pcl/visualization/cloud_viewer.h>
#include <Recognition/RecognitionData.h>

int main(int argc, char** argv){
  if(argc!=3){
    std::cerr << "Usage: " << argv[0] << " train/path camera/model\n";
    return -1;
  }

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  Recognition::RecognitionData mySister(argv[1], Camera::CameraModel::readFrom(fs["camera_model"]));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr axis(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::visualization::PCLVisualizer viewer("Object");
  viewer.addCoordinateSystem(1.0);
  while(1){
    viewer.spinOnce(100);
    std::cout << "Which object wanna draw? ";
    std::string name;
    std::cin >> name;
    viewer.removePointCloud("lol");
    if(name=="quit"){
      break;
    }
    double x,y,z,a,b,g;
    std::cout << "Enter x y z a b g: ";
    std::cin  >> x >> y >> z >> a >> b >> g;
    auto result=mySister.objectPointCloud(name);//,C5G::Pose::poseToTransform({x,y,z,a,b,g}));
    viewer.addPointCloud<pcl::PointXYZRGB>(result, "lol");
    while(!viewer.wasStopped()){
      viewer.spinOnce(100);
    }

  }
  return 0;
}
