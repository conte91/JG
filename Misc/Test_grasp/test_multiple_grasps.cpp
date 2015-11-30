#include <Utils/CvStorage.h>
#include <Gripper/GripperModel.h>
#include <Gripper/Types.h>
#include <Gripper/Grasper.h>
#include <C5G/Pose.h>

int main(int argc, char** argv){

  if(argc<10){
    std::cerr << "Usage: " << argv[0] << "/shapes/and/grasps/file /gripper/model/file what_to_grasp alpha beta gamma x y z [obj1 a1 b1 g1 x1 y1 n1 [...]] \n";
    return -1;
  }

  Gripper::ObjectsScene scene{};

  for(int startArgc=3; argc-startArgc>=7; startArgc+=7){
    double a,b,g,x,y,z;
    std::string objName(argv[startArgc]);
    a=::atof(argv[startArgc+1]);
    b=::atof(argv[startArgc+2]);
    g=::atof(argv[startArgc+3]);
    x=::atof(argv[startArgc+4]);
    y=::atof(argv[startArgc+5]);
    z=::atof(argv[startArgc+6]);
    Eigen::Affine3d objTransform=C5G::Pose{x,y,z,a,b,g}.toTransform();
    scene.emplace_back(objName, objTransform);
  }

  std::cout << "Hello! This program will search for the best grasp of " << scene[0].first << "From the following scene:\n";
  for(const auto& x :scene){
    std::cout << "----" << x.first << "----\n" << x.second.matrix() << "\n";
  }
  
  Gripper::ObjectDB objects;
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  fs["objects"] >> objects;

  Gripper::GripperModel gripper;
  cv::FileStorage gripperfs(argv[2], cv::FileStorage::READ);
  gripperfs["gripper"] >> gripper;

  auto result=gripper.getBestGrasp(scene[0].first,scene, objects);

  pcl::visualization::PCLVisualizer viewer("Grasps");
  
  for(size_t i=0; i<scene.size(); ++i){
    std::stringstream s;
    s << "Object " << i ;
    viewer.addPointCloud((scene[i].second*objects[argv[3]].myShape)->getPCSurface(50), s.str());
  }
  viewer.addPointCloud((result.second*gripper.shape())->getPCSurface(50), "Gripper");
  viewer.addCoordinateSystem(1.0);
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
  }
  return 0;
}
