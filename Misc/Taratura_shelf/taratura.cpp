#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <Camera/ImageProvider.h>
#include <Camera/OpenniStreamProvider.h>
#include <Camera/ImageConsumer.h>
#include <iostream>
#include <fstream>
#include <APC/Shelf.h>
#include <APC/Robot.h>

int main(int argc, char** argv){
  if(argc<4){
    std::cerr << "Usage: " << argv[0] << " server profile camera_poses_file\n";
    std::cerr  << "Example: " << argv[0] << " 172.22.178.102 CNTRLC5G_2200102 camera.poses\n";
    return -1;
  }

  std::string ip(argv[1]);
  std::string profile(argv[2]);
  std::string cameraFile(argv[3]);
  
  std::ifstream setupCamera(cameraFile.c_str());

  double x,y,z,a,b,g;
  setupCamera >> x >> y >> z >> a >> b >> g;
  C5G::Pose camera1({x,y,z,a,b,g});
  setupCamera >> x >> y >> z >> a >> b >> g;
  C5G::Pose camera2({x,y,z,a,b,g});
  if(setupCamera.fail()){
    std::cerr << "Error reading the cameras' poses. Try again after fixing it!";
    return -1;
  }

  std::cout << "I've read the following poses:\nCamera1: " << camera1 << "\nCamera2" << camera2 << "\n";

  /** This call also initializes the robot (unless false is passed as a third argument) */
  //C5G::C5G robot(argv[1], argv[2], false);
  boost::shared_ptr<Camera::ImageProvider> camera(new Camera::OpenniStreamProvider());
  APC::Robot robot(ip, profile, false, camera);

  Camera::Image photo_l, photo_r;
  try{
    robot.init();
    robot.moveCartesianGlobal(APC::Shelf::POSE_FOR_THE_PHOTOS);
    photo_l=robot.takePhoto();
    photo_r=robot.takePhoto();
  }
  catch(std::string e){
    std::cerr << e << "\n";
    return -2;
  }

  return 0;
}

