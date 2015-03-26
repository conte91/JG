#include <C5G/C5G.h>
#include <C5G/Pose.h>
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
  try{
    robot.init();
  }
  catch(std::string e){
    std::cerr << e << "\n";
    return -2;
  }

  
  while(1){
    double x, y, z, a, b, g;
    std::cout << "Enter pose x y z a b g: ";
    std::cin >> x >> y >> z >> a >> b >> g;
    C5G::Pose p(x,y,z,a,b,g);
    std::cout << "Moving to :" << p << "\n";
    try{
      robot.moveCartesianGlobal(p);
    }
    catch(std::string s){
      std::cerr << "Erm.. There has been an error.\n";
      std::cerr << "Details: " << s << "\n";
    }
  }
  return 0;
}

