#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <APC/Shelf.h>
#include <Camera/ImageConsumer.h>

namespace APC{
  void ScanBins(C5G::C5G& robot, Camera::ImageConsumer& photo){
    using APC::Shelf;
    using C5G::Pose;
    Pose p=Shelf::POSE;

    /** First, move globally into the safe pose of each bin */
    for(int i=0; i<Shelf::WIDTH; ++i){
      for(int j=0; j<Shelf::HEIGHT; ++j){
        /** Move to the bin */
        robot.moveCartesianGlobal(Shelf::getBinSafePose(i, j)+Shelf::POSE);
        photo.update();
        /** Go to a safe position - TODO needed?*/
        robot.moveCartesianGlobal(C5G::C5G::safePose);
      }
    }
  }
}
