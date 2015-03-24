#include <APC/Grasper.h>
namespace APC{
  Grasp getBestGrasp(std::string what, int row, int column){
    std::cout << "Implement me!\n";
    Pose none={0, 0, 0, 0 , 0, 0};
    Grasp theGraspThatIWantToReturn={what, none, none, 1.0, 0.0};
    return theGraspThatIWantToReturn;
  }
}
