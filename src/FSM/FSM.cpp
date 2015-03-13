#include <C5G/Pose.h>
#include <boost/thread.hpp>

void ScanFSM::run(){
  Pose p=Shelf.POSE;

  /** First, move globally into the safe pose of each bin */
  for(int i=0; i<Shelf.WIDTH, ++i){
    for(int j=0; j<Shelf.HEIGHT; ++j){
      /** Move to the bin */
      C5G::moveCartesianGlobal(Shelf.getBinSafePose(i, j)+Shelf.POSE);
      boost::this_thread.sleep(1);
      /** Go to a safe position - TODO needed?*/
      C5G::moveCartesianGlobal(C5G::safePose);
      boost::this_thread.sleep(1);
    }
  }
  
  /** Now, go to the position of the first bot and save the current position */
  C5G::moveCartesianGlobal(Shelf.getBinSafePose(i/2, j/2)+Shelf.getPose());

  C5G::setPosition(Shelf.getBinSafePose(i/2, j/2));
  
  /** Go in front of the shelf */
  C5G::moveCartesian(Shelf.SAFE_POSE);

  for(int i=0; i<Shelf.WIDTH, ++i){
    for(int j=0; j<Shelf.HEIGHT; ++j){
      /** Pass again all the bins */
      C5G::moveCartesian(Shelf.getBinSafePose(i, j));
      C5G::moveCartesian(Shelf.SAFE_POSE);
      boost::this_thread.sleep(1);
    }
  }
}
