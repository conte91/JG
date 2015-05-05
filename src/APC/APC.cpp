#include <boost/program_options.hpp>

#include <APC/APC.h>
#include <APC/Robot.h>
#include <C5G/C5G.h>
#include <C5G/Pose.h>
#include <C5G/Grasp.h>
#include <Camera/DummyConsumer.h>
#include <Camera/DummyProvider.h>
#include <Camera/OpenniProvider.h>
#include <Camera/OpenniStreamProvider.h>
#include <Camera/OpenniWaitProvider.h>
#include <APC/Order.h>
#include <APC/ReadWorkOrder.h>
#include <APC/ScanBins.h>
#include <APC/UpdateBins.h>
#include <APC/Shelf.h>
#include <APC/OrderBin.h>
#include <Parser/RobotData.h>
//#include <XnOpenNI.h>
//#include <openni2/OpenNI.h>

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

    std::string ip;
    std::string profile;

    namespace po=boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options() 
      ("help", "print help message")
      ("ip,i", po::value<std::string>(&ip)->required(), "IP address to connect to")
      ("profile,p", po::value<std::string>(&profile)->required(), "profile name")
      ("stream,s", po::value<std::string>(), "stream to a directory")
      ("wait,w" , "wait before taking shoots");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    Camera::ImageProvider::Ptr x;
    try{
      if(vm.count("stream")){
        x=Camera::ImageProvider::Ptr(new Camera::OpenniStreamProvider(vm["stream"].as<std::string>()));
      }
      else if(vm.count("wait")){
        x=Camera::ImageProvider::Ptr(new Camera::OpenNIWaitProvider());
      }
      else{
        x=Camera::ImageProvider::Ptr(new Camera::OpenNIProvider());
      }

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

    Camera::DummyConsumer img(x); 
    try{
      ScanBins(robot);
    }
    catch(std::string ex){
      std::cerr << "Error while scanning bins: " << ex << "\n";
      return -4;
    }

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
