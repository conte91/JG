#include <APC/APC.h>

#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <C5G/Grasp.h>
#include <Camera/DummyConsumer.h>
#include <Camera/DummyProvider.h>
#include <APC/Order.h>
#include <APC/ScanBins.h>
#include <APC/UpdateBins.h>
#include <APC/Shelf.h>
#include <APC/OrderBin.h>

namespace APC{
  int APC::main(){
    using C5G::Pose;
    using C5G::Grasp;
    using C5G::C5G;
    C5G robot;

    boost::shared_ptr<Camera::ImageProvider> x(new Camera::DummyProvider());
    Camera::DummyConsumer img(x); 
    APC::ScanBins(robot, img);

    APC::OrderStatus orderBin;
    for(i: InterProcessCommunication::RobotData.getInstance().workOrder){
      orderBin.push_back(new order(i));
    }

    try{
      while(!orderBin.empty()){
        APC::updateBins(orderBin);
        APC::Order x=orderBin.top();
        orderBin.pop();
        if(x.grasp.score < APC::Order::MIN_SCORE_WE_CAN_MANAGE){
          throw std::string("Remaining items are too much difficult to take!");
        }
        std::cout << "Trying to grasp item: " << x.object << std::endl;
        Grasp& todoGrasp=x.grasp;
        robot.moveCartesianGlobal(APC::Shelf::getBinSafePose(x.bin[0], x.bin[1]));
        robot.setZero();
        robot.executeGrasp(todoGrasp);
        Pose origin={0, 0, 0, 0, 0, 0};
        robot.moveCartesian(origin);
        /** TODO metterli bene */
        origin.z+=APC::OrderBin::HEIGHT+0.1;
        robot.moveCartesianGlobal(APC::OrderBin::POSE+origin);
        robot.setGripping(0);
      }
    }
    catch(std::string s){
      std::cerr << s << "\n"; 
    }

    return 0;
  }
}
