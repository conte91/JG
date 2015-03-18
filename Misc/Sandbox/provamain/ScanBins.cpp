#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <APC/Shelf.h>

void ScanBins(C5G& robot, ImageConsumer& photo){
  using APC::Shelf;
  Pose p=Shelf.POSE;

  /** First, move globally into the safe pose of each bin */
  for(int i=0; i<Shelf.WIDTH, ++i){
    for(int j=0; j<Shelf.HEIGHT; ++j){
      /** Move to the bin */
      C5G::moveCartesianGlobal(Shelf.getBinSafePose(i, j)+Shelf.POSE);
      photo.Update();
      /** Go to a safe position - TODO needed?*/
      C5G::moveCartesianGlobal(C5G::safePose);
    }
  }
}
