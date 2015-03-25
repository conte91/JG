#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <C5G/C5G.h>
#include <C5G/Grasp.h>

namespace C5G{
  void C5G::moveCartesian(const Pose& p){
    std::cout << "Relative movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
  }

  const Pose C5G::safePose(0.3, 0, 0.7, 0, 0, 0);
  void C5G::moveCartesianGlobal(const Pose& p){
    std::cout << "Global movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
  }

  void C5G::init(){
    std::cout << "Initing the system..\nConnecting to IP address: " << _ip << "\nSystem ID: " << _sys_id << "\n";
    std::cout << "Done.\n";
  }

  C5G::C5G(const std::string& ip, const std::string& sys_id, bool mustinit)
    :
      _ip(ip),
      _sys_id(sys_id){
        if(mustinit){
          init();
        }
      }

  void C5G::standby(){
    std::cout << "Goodbye, cruel world..\n";
    std::cout << "I'm leaving you today..\n";
    std::cout << "Goodbye..\n";
    std::cout << "Goodbye..\n";
    std::cout << "Goodbye.\n";
  }
  void C5G::setGripping(double strength){
    std::cout << "Closing the plier with strength " << strength << "\n";
  }
  void C5G::executeGrasp(const Grasp& g){
    moveCartesian(g.approach);
    moveCartesian(g.grasp);
    setGripping(g.forceMin);
    moveCartesian(g.approach);
  }
  C5G::~C5G(){
    standby();
  }
}

