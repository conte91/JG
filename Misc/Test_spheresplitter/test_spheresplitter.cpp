#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <Recognition/SphereSplitter.h>

int main(int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  if(argc!=2){
    std::cout << "Specify minimum n points\n";
  }

  auto myPts=Recognition::SphereSplitter(atoi(argv[1])).points();

  std::cout << "Number of verteces: " << myPts.size() << "\n";
  std::cout << "Points: \n[\n";
  for(const auto& x: myPts){
    std::cout << "(" <<  x  << ")\n";
    points->push_back({x[0],x[1],x[2]});
  }

  pcl::visualization::PCLVisualizer viewer("Scene's PCL");
  viewer.addCoordinateSystem(1.0);
  viewer.addPointCloud<pcl::PointXYZ>(points, "scene");
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
  }
}

