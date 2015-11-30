#include <Utils/CvStorage.h>
#include <Gripper/GripperModel.h>
#include <Gripper/Types.h>

int main(int argc, char** argv){

  if(argc!=4){
    std::cerr << "Usage: " << argv[0] << "/shapes/and/grasps/file /gripper/model/file what_to_grasp_in_origin\n";
    return -1;
  }

  std::cout << "Hello! This program will search for the best grasp of a known object located into the origin!\n";
  
  Gripper::ObjectDB objects;
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  fs["objects"] >> objects;

  Gripper::GripperModel gripper;
  cv::FileStorage gripperfs(argv[2], cv::FileStorage::READ);
  gripperfs["gripper"] >> gripper;

  auto result=gripper.getBestGrasp(argv[3], {std::make_pair(argv[3], Eigen::Affine3d::Identity())}, objects);

  pcl::visualization::PCLVisualizer viewer("Grasps");
  
  viewer.addPointCloud(objects[argv[3]].myShape->getPCSurface(50), "Object");
  viewer.addPointCloud((result.second*gripper.shape())->getPCSurface(50), "Gripper");
  viewer.addCoordinateSystem(1.0);
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
  }
  return 0;
}

  

