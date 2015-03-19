#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <C5G/C5G.h>
#include <Log/Log.h>
#include <eORL.h>

namespace C5G{
  void C5G::moveCartesian(const Pose& p){
    Log::out << "Relative movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
  }

  void C5G::moveCartesianGlobal(const Pose& p){
    Log::out << "Global movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
  }

  void C5G::init(){
    Log::out << "Initing the system..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(3));
    Log::out << "Done.\n";
  }

  void C5G::standby(){
    Log::out << "Goodbye, cruel world..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    Log::out << "I'm leaving you today..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    Log::out << "Goodbye..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    Log::out << "Goodbye..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    Log::out << "Goodbye.\n";
  }
}
