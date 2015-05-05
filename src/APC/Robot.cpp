#include <APC/Robot.h>
#include <APC/Shelf.h>
#include <Camera/ImageProvider.h>
namespace APC{
    void Robot::moveToBin(int row, int column){
      moveCartesianGlobal(Shelf::getBinSafePose(row, column));
    }
    Robot::Robot(const std::string& ip, const std::string& sys_id, bool mustInit, const Camera::ImageProvider::Ptr& camera)
    :
      C5G::C5G(ip, sys_id, mustInit),
      _provider(camera)
     {
      }

    Camera::Image Robot::takePhoto(){
      return this->_provider->getFrame();
    }

    void Robot::executeGrasp(const ::C5G::Grasp& g){
      std::cout << "Executing grasp for object " << g.object << "\n";
      moveCartesianGlobal(Shelf::getBinSafePose(g.row, g.column));
      moveCartesianGlobal(g.approach.whichIsRelativeTo(Shelf::CAMERA_POSE));
      moveCartesianGlobal(g.grasp.whichIsRelativeTo(Shelf::CAMERA_POSE));
      setGripping(g.forceMax);
    }

}
