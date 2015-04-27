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
    for(int j=0; j<Shelf::HEIGHT; ++j){
      for(int i=0; i<Shelf::WIDTH; ++i){
        /** First, move globally into the safe pose of each bin */
        Pose whereToGo=Shelf::getBinSafePose(i, j)+Shelf::POSE;
        whereToGo.alpha=0;
        whereToGo.beta=1.57;
        whereToGo.gamma=0;

        /** Go to the safe pose */
        Pose p=C5G::C5G::safePose();
        robot.moveCartesianGlobal(p);

        /** Approach the shelf */
        p.x=whereToGo.x;
        p.y=whereToGo.y;
        robot.moveCartesianGlobal(p);

        /** Move to the bin */
        robot.moveCartesianGlobal(whereToGo);
        rData.setDirty(i, j);

        /** Return to the approach position */
        robot.moveCartesianGlobal(p);
      }
    }
  }
}
