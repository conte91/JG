#include <APC/Grasper.h>
namespace APC{
  C5G::Grasp getBestGrasp(std::string what, int row, int column){
    std::cout << "Implement me!\n";
    C5G::Pose none(0, 0, 0, 0 , 0, 0);
    C5G::Grasp theGraspThatIWantToReturn={what, none, none, 1.0, 0.0};
    return theGraspThatIWantToReturn;
  }
}
