#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <C5G/C5G.h>
#include <C5G/Grasp.h>

namespace C5G{
  void C5G::moveCartesian(const Pose& p){
    std::cout << "Relative movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
  }

  const Pose C5G::safePose={0.3, 0, 0.7, 0, 0, 0};
  void C5G::moveCartesianGlobal(const Pose& p){
    std::cout << "Global movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
  }

  void C5G::init(const std::string& ip, const std::string& sysID){
    std::cout << "Initing the system..\nConnecting to IP address: " << ip << "\nSystem ID: " << sysID << "\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(3));
    std::cout << "Done.\n";
  }

  void C5G::standby(){
    std::cout << "Goodbye, cruel world..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "I'm leaving you today..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "Goodbye..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "Goodbye..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "Goodbye.\n";
  }
  void C5G::setGripping(double strength){
    std::cout << "Closing the plier with strength " << strength << "\n";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
  void C5G::executeGrasp(const Grasp& g){
    moveCartesian(g.approach);
    moveCartesian(g.grasp);
    setGripping(g.force);
    moveCartesian(g.approach);
  }
  C5G::~C5G(){
    standby();
  }
}

