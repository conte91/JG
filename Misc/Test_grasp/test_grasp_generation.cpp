#include <algorithm>
#include <Gripper/Types.h>
#include <Gripper/GraspPose.h>
#include <Utils/CvStorage.h>

int main(int argc, char** argv){

  if(argc!=3){
    std::cerr << "Usage: " << argv[0] << "/shapes/and/grasps/file what_to_visualize\n";
    return -1;
  }

  Gripper::ObjectDB objects;
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);

  fs["objects"] >> objects;

  std::string toVisualize(argv[2]);
  pcl::visualization::PCLVisualizer viewer("Grasps");

  viewer.addPointCloud(objects[toVisualize].myShape->getPCSurface(50));
  for(const auto& x :objects[toVisualize].myGrasps){
    x.drawToViewer(viewer, 0.05);
  }

  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
  }
  return 0;
}
