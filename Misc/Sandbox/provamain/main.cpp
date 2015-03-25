#include <C5G/C5G.h>
#include "ScanBins.h"
#include <Camera/DummyConsumer.h>
#include <Camera/DummyProvider.h>

int main(int argc, char** argv){
  using C5G::C5G;
  if(argc<3){
    std::cerr << "Usage: " << argv[0] << " server profile\n";
    std::cerr  << "Example: " << argv[0] << " 172.22.178.102 CNTRLC5G_2200102\n";
    return -1;
  }

  std::string ip(argv[1]);
  std::string profile(argv[2]);
  C5G robot(ip, profile, false);
  try{
    robot.init();
  }
  catch(std::string ex){
    std::cerr << ex << "\n";
    return -2;
  }

  boost::shared_ptr<Camera::ImageProvider> x(new Camera::DummyProvider());
  Camera::DummyConsumer img(x); 
  ScanBins(robot, img);
  return 0;

}
