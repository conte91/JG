#include <C5G/C5G_socket.h>
#include <sstream>

namespace C5G{

  const std::string C5G::CONNECTION_PORT("1101");
  void C5G::setZero(){
    setPosition(Pose(0, 0, 0, 0, 0, 0));
    std::cout << "I'm now at zero.\n";
  }

  void C5G::setPosition(const Pose& p){
    if(_currentMovementMode==MOVING_GLOBAL){
      /** We were in global mode; save current position in order to restore it when needed */
      _lastGlobalPose={0,0,0,0,0,0};
      _currentMovementMode=MOVING_RELATIVE;
    }
    std::cout << "Setting the position to: " << p << "\n";
  }

  void C5G::moveCartesianGlobal(const Pose& p){

    std::cout << "Global movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
    _connectionToRobot << "L <" << p.x << "," << p.y << "," << p.z << "," << p.alpha << "," << p.beta << "," << p.gamma << "," << ""/**TODO?*/<< ">"
    _connectionToRobot.flush();
    std::string s;
    _connectionToRobot >> s;
    if(s!="ACK"){
      throw std::string("No ACK received\n");
    }
    std::cout << "Movement ended.\n";
  }

  /** TODO CHECK THIS!*/
  const Pose C5G::safePose(){
    static Pose theSafePose(0.3, 0, 0.9, 0, 1.57, 0);
    return theSafePose;
  }
  void C5G::moveCartesian(const Pose& p){
    std::cout << "Implement me!\n";
    std::cout << "Relative movement to (" << p.x << ", " << p.y << ", " << p.z << ")\nOrientation: (" << p.alpha << ", " << p.beta << ", " << p.gamma << "\n";
  }

  void C5G::init(){
    std::cout << "Connecting to " << _ip << " on port " << CONNECTION_PORT << "..\n";
    _connectionToRobot.connect(_ip, CONNECTION_PORT);
    if(!_connectionToRobot){
      throw std::string("Could not connect to robot!\n");
    }
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

  C5G::C5G(const std::string& ip, const std::string& sys_id, bool mustInit)
    :
    _ip(ip),
    _sys_id(sys_id),
    _currentMovementMode(MOVING_GLOBAL)
  {
    if(mustInit){
      init();
    }
  }
  C5G::~C5G(){
    standby();
  }

  void C5G::setGripping(double strength){
    std::cout << "Closing the plier with strength " << strength << "\n";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
}
