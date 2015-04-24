#include <APC/APC.h>
#include <APC/Robot.h>
#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <C5G/Grasp.h>
#include <Camera/DummyConsumer.h>
#include <Camera/DummyProvider.h>
#include <Camera/OpenniProvider.h>
#include <APC/Order.h>
#include <APC/ReadWorkOrder.h>
#include <APC/ScanBins.h>
#include <APC/UpdateBins.h>
#include <APC/Shelf.h>
#include <APC/OrderBin.h>
#include <Parser/RobotData.h>

namespace APC{
  /** Base idea:
   * Start with scanning all bins. In this way each bin has a recorded image, and each order is marked as "dirty".
   * while(!finished):
   *    Sort the objects which still have to be taken by score of their best grasp, after updating all the orders for which the best grasp score could have changed. 
   *    Take the best one,
   *    go in front of the bin (basic point from which the best grasp has been computed)
   *    start executing the grasp
   *    go back in front of the bin
   *    Bring the object into the order bin
   *    (go to a safe position)
   *
   */
  int APC::main(int argc, char** argv){
    using C5G::Pose;
    using C5G::Grasp;
    using C5G::C5G;
    if(argc<3){
      std::cerr << "Usage: " << argv[0] << " server profile\n";
      std::cerr  << "Example: " << argv[0] << " 172.22.178.102 CNTRLC5G_2200102\n";
      return -1;
    }

    std::string ip(argv[1]);
    std::string profile(argv[2]);
    Camera::ImageProvider::Ptr x;
    try{
      x=Camera::ImageProvider::Ptr(new Camera::OpenNIProvider());
    }
    catch(std::string what){
      std::cerr << "Error: " << what << ".\n Type OK to continue working with a dummy (NULL) provider.\n";
      std::string aaa;
      std::cin >> aaa;
      if(aaa=="OK"){
        x=Camera::ImageProvider::Ptr(new Camera::DummyProvider());
      }
      else{
        return -1;
      }
    }

    //Camera::DummyConsumer img(x); 
    Robot robot(ip, profile, false, x);
    try{
      robot.init();
    }
    catch(std::string ex){
      std::cerr << ex << "\n";
      return -2;
    }


    ScanBins(robot);

    OrderStatus orderBin;
    InterProcessCommunication::RobotData& rData=InterProcessCommunication::RobotData::getInstance();

    std::cout << "Contents of the bins:\n" << rData << "\n";
    readWorkOrder();

    std::cout << "After loading:\n" << rData << "\n";
    auto workOrder=rData.getWorkOrder();

    orderBin=workOrder;
    std::cout << "Items to take: " << orderBin << "\n";

    try{
      while(!orderBin.empty()){
        std::cout << "Updating bins..\n";
        updateBins(orderBin, robot);
        std::cout << "Finished updating.\n";
        std::cout << "Remaining order bin: ----------\n" << orderBin << "\n---------\n";
        Order x=orderBin.top();
        orderBin.pop();
        std::cout << "Best order: " << x << "----------\n";
        if(x.grasp.score < Order::MIN_SCORE_WE_CAN_MANAGE){
          throw std::string("Remaining items are too much difficult to take!");
        }
        std::cout << "Trying to grasp item: " << x.object << std::endl;
        Grasp todoGrasp=x.grasp;
        robot.moveCartesianGlobal(Shelf::getBinSafePose(x.bin[0], x.bin[1]));
        robot.setZero();
        robot.executeGrasp(todoGrasp);
        Pose origin(0, 0, 0, 0, 0, 0);
        robot.moveCartesian(origin);
        robot.moveCartesianGlobal(OrderBin::POSE+Pose(0, 0, OrderBin::HEIGHT+0.1, 0, 0, 0));
        robot.setGripping(0);
      }
    }
    catch(std::string s){
      std::cerr << s << "\n"; 
    }

    return 0;
  }
}
