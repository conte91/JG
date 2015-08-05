#include <pcl/visualization/cloud_viewer.h>
#include <Recognition/RecognitionData.h>

int main(int argc, char** argv){
  if(argc!=3){
    std::cerr << "Usage: " << argv[0] << " train/path camera/model\n";
  }

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  Recognition::RecognitionData mySister(argv[1], Camera::CameraModel::readFrom(fs["camera_model"]));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr axis(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(double i=0; i<0.5; i+=0.02){
    pcl::PointXYZRGB x,y,z;
    x.x=i;
    x.y=0;
    x.z=0;
    x.r=255;
    x.g=0;
    x.b=0;

    y.x=0;
    y.y=i;
    y.z=0;
    y.r=0;
    y.g=255;
    y.b=0;
    z.x=0;
    z.y=0;
    z.z=i;
    z.r=0;
    z.g=0;
    z.b=255;
    axis->push_back(x);
    axis->push_back(y);
    axis->push_back(z);
  }
  pcl::visualization::PCLVisualizer viewer("Object");
  viewer.addPointCloud<pcl::PointXYZRGB> (axis, "axis");
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
    auto result=mySister.objectPointCloud(name,{x,y,z,a,b,g});
    viewer.addPointCloud<pcl::PointXYZRGB>(result, "lol");
  }
  return 0;
}
