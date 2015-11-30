#include <Utils/CvStorage.h>
#include <Gripper/GripperModel.h>
#include <Gripper/Types.h>
#include <C5G/Pose.h>

int main(int argc, char** argv){

  if(argc!=10){
    std::cerr << "Usage: " << argv[0] << "/shapes/and/grasps/file /gripper/model/file what_to_grasp alpha beta gamma x y z\n";
    return -1;
  }

  double a,b,g,x,y,z;
  
  a=::atof(argv[4]);
  b=::atof(argv[5]);
  g=::atof(argv[6]);
  x=::atof(argv[7]);
  y=::atof(argv[8]);
  z=::atof(argv[9]);

  Eigen::Affine3d objTransform=C5G::Pose{x,y,z,a,b,g}.toTransform();

  std::cout << "Hello! This program will search for the best grasp of a known object located at:\n" << objTransform.matrix() << "\n";
  
  Gripper::ObjectDB objects;
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  fs["objects"] >> objects;

  Gripper::GripperModel gripper;
  cv::FileStorage gripperfs(argv[2], cv::FileStorage::READ);
  gripperfs["gripper"] >> gripper;

  auto result=gripper.getBestGrasp(argv[3], {std::make_pair(argv[3], objTransform)}, objects);

  pcl::visualization::PCLVisualizer viewer("Grasps");
  
  viewer.addPointCloud((objTransform*objects[argv[3]].myShape)->getPCSurface(50), "Object");
  viewer.addPointCloud((result.second*gripper.shape())->getPCSurface(50), "Gripper");
  viewer.addCoordinateSystem(1.0);
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
  }
  return 0;
}
