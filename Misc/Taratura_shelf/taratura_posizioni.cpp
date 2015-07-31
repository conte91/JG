#include <iostream>
#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <APC/Shelf.h>

int main(int argc, char** argv){
  if(argc<3){
    std::cerr << "Usage: " << argv[0] << " server profile\n";
    std::cerr  << "Example: " << argv[0] << " 172.22.178.102 CNTRLC5G_2200102\n";
    return -1;
  }
  std::string ip(argv[1]);
  std::string profile(argv[2]);

  C5G::C5G robot(ip, profile, false);
  try{
    robot.init();
  }
  catch(std::string e){
    std::cerr << e << "\n";
    return -2;
  }
  bool haveFinished=false;
  while(!haveFinished){
    std::string s;

    std::cout << "I'm going to corner of bin D. Align the robot consequently..\n";
    robot.moveCartesianGlobal(robot.safePose());
    robot.moveCartesianGlobal(APC::Shelf::getBinCornerPose(1,0).whichIsRelativeTo(APC::Shelf::POSE));
    std::cout << "Type something when done..\n";
    std::cin >> s;

    std::cout << "Now I'm going to corner of bin L. Again, align..\n";
    robot.moveCartesianGlobal(robot.safePose());
    robot.moveCartesianGlobal(APC::Shelf::getBinCornerPose(3,2).whichIsRelativeTo(APC::Shelf::POSE));
    std::cout << "Type something when done..\n";
    std::cin >> s;

    std::cout << "Going back to corner of bin D.\n";
    robot.moveCartesianGlobal(robot.safePose());
    robot.moveCartesianGlobal(APC::Shelf::getBinCornerPose(1,0).whichIsRelativeTo(APC::Shelf::POSE));
    std::cout << "Are you satisfied with this movement (y/n)?\n";
    std::cin >> s;
    if(s=="y"){
      haveFinished=true;
    }
  }
  return 0;
}
