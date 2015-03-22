#include <C5G/C5G.h>
#include "ScanBins.h"
#include <Camera/DummyConsumer.h>
#include <Camera/DummyProvider.h>
#include <APC/Order.h>

int main(){
  using C5G::C5G;
  C5G robot;

  boost::shared_ptr<Camera::ImageProvider> x(new Camera::DummyProvider());
  Camera::DummyConsumer img(x); 
  ScanBins(robot, img);

  APC::OrderStatus orderBin;

  try{
    while(!orderBin.empty()){
      APC::updateBin(orderBin);
      APC::Order& x=orderBin.top();
      orderBin.pop();
      if(x.score < APC::MIN_SCORE_WE_CAN_MANAGE){
        throw std::string("Remaining items are too much difficult to take!");
      }
      std::cout << "Trying to grasp item: " << x.object << std::endl;
      Grasp& todoGrasp=x.grasp;
      robot.moveCartesianGlobal(Shelf.getBinSafePose(x.binX, x.binY));
      robot.setZero();
      robot.executeGrasp(todoGrasp);
      robot.moveCartesian(C5G::Pose(0,0,0,0,0,0));
      robot.moveCartesianGlobal(OrderBin.POSE);
      robot.setGripping(0);
    }
  }
  catch(std::string s){
    std::cerr << s << "\n"; 
  }

  return 0;
}
