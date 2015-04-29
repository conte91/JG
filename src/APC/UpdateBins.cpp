#include <APC/UpdateBins.h>
#include <APC/Robot.h>
#include <APC/Grasper.h>
#include <Parser/RobotData.h>
#include <Camera/Recognition.h>

namespace APC{
  void updateBins(OrderStatus& order, Robot& robot){
    /** Dunque..
     * We have to update the data on all the bins with the score
     * of the best grasp!
     *
     * This is done so that the robot can choose to take the easier object every time :)
     *
     * We build a new order queue, recomputing the best grasp for all 
     * the objects for which the best grasp could have changed.
     */

    OrderStatus neworder;

    InterProcessCommunication::RobotData& r=InterProcessCommunication::RobotData::getInstance();

    while(!order.empty()){
      /** Take a new order and recompute it if needed; then put it again into the quueueueueueueueueueueueueueueueueeueueue*/
      Order x=order.top();
      order.pop();
      std::cout << "<------------------------------->\n";

      if(r.isDirty(x.bin[0], x.bin[1])){
        robot.moveToBin(x.bin[0], x.bin[1]);
        //SUGGESTION: mostrare foto di bin imshow
        Image photo = robot.takePhoto()
        r.setPhoto(x.bin[0], x.bin[1],photo);
        r.demoViewer.showImage(photo);
        r.demoViewer.setTitle("new Data");
        Camera::updateGiorgio(x.bin[0], x.bin[1]);
      }
       
      /** We moved things, lets'a update the bin content */
      std::cout << "Computing best grasp for order " << x << "\n";
      x.grasp=APC::getBestGrasp(x.object, x.bin[0], x.bin[1]);
      neworder.push(x);
    }
    order=neworder;
  }

}
