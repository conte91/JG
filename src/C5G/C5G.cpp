#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <C5G/C5G.h>
#include <Log/Log.h>
#include <eORL.h>

namespace C5G{
  void C5G::moveCartesianGlobal(const Pose& p){
    ORL_cartesian_position  target_pos;
    target_pos.x=p.x;
    target_pos.y=p.y;
    target_pos.z=p.z;
    target_pos.a=p.alpha;
    target_pos.e=p.beta;
    target_pos.r=p.gamma;
    Log::out << "Global movement to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")\nOrientation: (" << target_pos.a << ", " << target_pos.e << ", " << target_pos.r << "\n";
  }
  void C5G::moveCartesian(const Pose& p){
    ORL_cartesian_position  target_pos;
    target_pos.x=p.x;
    target_pos.y=p.y;
    target_pos.z=p.z;
    target_pos.a=p.alpha;
    target_pos.e=p.beta;
    target_pos.r=p.gamma;
    Log::out << "Relative movement to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")\nOrientation: (" << target_pos.a << ", " << target_pos.e << ", " << target_pos.r << "\n";
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
