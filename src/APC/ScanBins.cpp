#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <APC/Shelf.h>
#include <Parser/RobotData.h>
#include <Camera/ImageConsumer.h>
#include <Parser/RobotData.h>

namespace APC{
  void ScanBins(C5G::C5G& robot){
    using APC::Shelf;
    using C5G::Pose;
    Pose p=Shelf::POSE;

    InterProcessCommunication::RobotData& rData=InterProcessCommunication::RobotData::getInstance();
    /** First, move globally into the safe pose of each bin */
    for(int i=0; i<Shelf::WIDTH; ++i){
      for(int j=0; j<Shelf::HEIGHT; ++j){
        /** Move to the bin */
        Pose whereToGo=Shelf::getBinSafePose(i, j)+Shelf::POSE;
        whereToGo.alpha=0;
        whereToGo.beta=1.57;
        whereToGo.gamma=0;
        robot.moveCartesianGlobal(whereToGo);
        rData.setDirty(i, j);
        /** Go to a safe position - TODO needed?*/
        robot.moveCartesianGlobal(C5G::C5G::safePose());
      }
    }
  }
}
