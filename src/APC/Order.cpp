#include <APC/Order.h>
#include <iostream>
#include <Parser/RobotData.h>

namespace APC {
  bool operator<(const Order& left, const Order& right){
    return (left.grasp.score < right.grasp.score);
  }

  Order::Order(const std::string& objName){
    object=objName;
    bin[0]=0;
    bin[1]=0;
  }
  const double Order::MIN_SCORE_WE_CAN_MANAGE=0.5;

  std::ostream& operator<< (std::ostream& out, const APC::Order& in){

    std::cout << in.object << " @bin (" << in.bin[0] << "," << in.bin[1] << ") -> @bin " << InterProcessCommunication::RobotData::xyToName(in.bin[0], in.bin[1]) << " score=" << in.grasp.score << "\n";
    return out;
  }
  std::ostream& operator<< (std::ostream& out, const APC::OrderStatus& in){
    std::cout << "INSIZE: " << in.size() << "\n";
    APC::OrderStatus tmp=in;
    int counter=0;
    while(!tmp.empty()){
      std::cout << counter++;
      APC::Order v=tmp.top();
      out << v.object << " (bin " << v.bin[0] << "," << v.bin[1] << " -> bin " << InterProcessCommunication::RobotData::xyToName(v.bin[0], v.bin[1]) <<  "\n";
      tmp.pop();
    }
    return out;
  }

}
