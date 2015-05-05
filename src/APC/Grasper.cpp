#include <fstream>
#include <iostream>
#include <C5G/Pose.h>
#include <C5G/Grasp.h>
#include <APC/Grasper.h>
#include <Parser/RobotData.h>
namespace APC{
  C5G::Grasp getBestGrasp(std::string what, int row, int column){
    using InterProcessCommunication::RobotData;
    std::cout << "Computing the best grasp for " << what << "\n";
    RobotData& r=RobotData::getInstance();
    {
      std::ofstream f("/tmp/robot.data");
      for(int i=0; i<RobotData::MAX_ITEM_N; ++i){
        std::string item=r.getBinItem(row, column, i);
        C5G::Pose thePose= r.getObjPose(row, column, i);
        if(item!=""){
          f << item << " " << thePose.x << " " << thePose.y << " " << thePose.z << " " << thePose.alpha << " " << thePose.beta << " " << thePose.gamma << "\n";
        }
      }
    }
    C5G::Pose approach, gPose;
    double theForceMayBeWithYou;
    double score;
    system(("python ./gripper.py "+what).c_str());
    {
      std::ifstream resultFile("/tmp/grasp.result");
      resultFile >> approach.x >> approach.y >> approach.z >> approach.alpha >> approach.beta >> approach.gamma ;
      resultFile >> gPose.x >> gPose.y >> gPose.z >> gPose.alpha >> gPose.beta >> gPose.gamma ;
      resultFile >> theForceMayBeWithYou >> score;
    }
#if 0
    approach.x=1.50;
    approach.y=0.30;
    approach.z=0.5;

#endif
    C5G::Grasp x({what, row, column, approach, gPose, theForceMayBeWithYou, score});
    //std::cout << "\nFinished computing grasp. New grasp: " << approach //<< "," << gPose << "," << theForceMayBeWithYou << ","<< score << "\n";
    return x;

#if 0
    std::cout << "Implement me!\n";
    C5G::Pose none(0.1, 0, 0, 0 , 0, 0);
    C5G::Grasp theGraspThatIWantToReturn={what, none, none, 1.0, 0.0};
    return theGraspThatIWantToReturn;
#endif
  }
}
