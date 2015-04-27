#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <APC/ScanBins.h>
#include <Camera/ImageProvider.h>
#include <Camera/ImageConsumer.h>
#include <Camera/DummyProvider.h>
#include <Camera/DummyConsumer.h>
#include <iostream>

int main(int argc, char** argv){
  if(argc<3){
    std::cerr << "Usage: " << argv[0] << " server profile\n";
    std::cerr  << "Example: " << argv[0] << " 172.22.178.102 CNTRLC5G_2200102\n";
    return -1;
  }

  std::string ip(argv[1]);
  std::string profile(argv[2]);
  
  /** This call also initializes the robot (unless false is passed as a third argument) */
  C5G::C5G robot(argv[1], argv[2], false);
  boost::shared_ptr<Camera::ImageProvider> x(new Camera::DummyProvider());
  Camera::DummyConsumer img(x); 
  try{
    robot.init();
    APC::ScanBins(robot);
  }
  catch(std::string e){
    std::cerr << e << "\n";
    return -2;
  }

  return 0;
}

