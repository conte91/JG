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
#include <Parser/RobotData.h>

namespace APC{
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
    C5G robot(ip, profile, false);
    try{
      robot.init();
    }
    catch(std::string ex){
      std::cerr << ex << "\n";
      return -2;
    }


    boost::shared_ptr<Camera::ImageProvider> x(new Camera::DummyProvider());
    Camera::DummyConsumer img(x); 
    ScanBins(robot, img);

    OrderStatus orderBin;
    InterProcessCommunication::RobotData& rData=InterProcessCommunication::RobotData::getInstance();
    std::vector<std::string> workOrder =rData.getWorkOrder();
    for(std::vector<std::string>::iterator i=workOrder.begin(); i!=workOrder.end(); ++i){
      orderBin.push(Order(*i));
    }

    try{
      while(!orderBin.empty()){
        updateBins(orderBin);
        Order x=orderBin.top();
        orderBin.pop();
        if(x.grasp.score < Order::MIN_SCORE_WE_CAN_MANAGE){
          throw std::string("Remaining items are too much difficult to take!");
        }
        std::cout << "Trying to grasp item: " << x.object << std::endl;
        Grasp& todoGrasp=x.grasp;
        robot.moveCartesianGlobal(Shelf::getBinSafePose(x.bin[0], x.bin[1]));
        robot.setZero();
        robot.executeGrasp(todoGrasp);
        Pose origin(0, 0, 0, 0, 0, 0);
        robot.moveCartesian(origin);
        /** TODO metterli bene */
        origin.z+=OrderBin::HEIGHT+0.1;
        robot.moveCartesianGlobal(OrderBin::POSE+origin);
        robot.setGripping(0);
      }
    }
    catch(std::string s){
      std::cerr << s << "\n"; 
    }

    return 0;
  }
}
