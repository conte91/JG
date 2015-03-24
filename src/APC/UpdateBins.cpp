#include <APC/UpdateBins.h>
#include <Parser/RobotData.h>

namespace APC{
  void updateBins(OrderStatus& order){
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
      /** Take a new order and recompute it if needed; then put it again into the queueueue*/
      Order x=order.top();
      order.pop();

      if(r.isDirty(x.bin[0], x.bin[1])){
        /**
         * moveToBin(x.bin[0], x.bin[1])
         * RobotData.bin[].photo=takePhoto();
         * updateGiorgio()
         */
          /** We moved things, lets'a update the bin content */
          x.grasp=getBestGrasp(x.object, x.bin[0], x.bin[1]);
      }
      neworder.push(x);
    }
    order=neworder;

}
