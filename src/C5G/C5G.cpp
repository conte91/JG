#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <C5G/C5G.h>
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
    std::cout << "Global movement to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")\nOrientation: (" << target_pos.a << ", " << target_pos.e << ", " << target_pos.r << "\n";
  }
  /** TODO CHECK THIS!*/
  const Pose C5G::safePose={0.3, 0, 0.7, 0, 0, 0};
  void C5G::moveCartesian(const Pose& p){
    ORL_cartesian_position  target_pos;
    target_pos.x=p.x;
    target_pos.y=p.y;
    target_pos.z=p.z;
    target_pos.a=p.alpha;
    target_pos.e=p.beta;
    target_pos.r=p.gamma;
    std::cout << "Relative movement to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")\nOrientation: (" << target_pos.a << ", " << target_pos.e << ", " << target_pos.r << "\n";
  }

  void C5G::init(){
    std::cout << "Initing the system..\n";
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

  void C5G::executeGrasp(const Grasp& g){
    std::cout << "Executing grasp for object " << g.object << "\n";
    //moveCartesianGlobal(Shelf.getBinSafePose(

  }

  void C5G::setGripping(double strength){
    std::cout << "Closing the plier with strength " << strength << "\n";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
}
