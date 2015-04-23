#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <Camera/Image.h>
#include <Parser/RobotData.h>
#include <C5G/Grasp.h>

namespace InterProcessCommunication{

  std::ostream& operator<< (std::ostream& os, const InterProcessCommunication::RobotData& r){
    for(int i=0; i<12; ++i){
      os << "Bin " << (char) (i+'A') << ": ";
      for(int item=0; item<RobotData::MAX_ITEM_N; ++item){
        std::string s=r.shelf.bins[i].object[item];
        os << r.shelf.bins[i].object[item];
        os << ",";
      }
      os << "\n";
    }
    return os;
  }

  int RobotData::getCurrentRow(){
    return _row;
  }

  int RobotData::getCurrentColumn(){
    return _column;
  }

  void RobotData::setCurrentRow(int row){
    _row=row;
  }

  void RobotData::setCurrentColumn(int column){
    _column=column;
  }

  RobotData& RobotData::getInstance() {
    static RobotData instance; // Guaranteed to be destroyed.
    return instance;
  }

  std::string RobotData::getBinItem(int row, int column, int item){
    return this->shelf.bins[xyToBin(row,column)].object[item];
  }

  void RobotData::setBinItem(int row,int column,int item,const std::string& val){
    this->shelf.bins[xyToBin(row,column)].object[item] = val;
  }
  C5G::Pose RobotData::getObjPose(int row, int column, int item) const{
    return this->shelf.bins[xyToBin(row,column)].objPose[item];
  }

  void RobotData::setObjPose(int row,int column,int item,const C5G::Pose& val){
    this->shelf.bins[xyToBin(row,column)].objPose[item] = val;
  }

  int RobotData::xyToBin(int row, int column){
    return (row*COL_N)+column;
  }

  void RobotData::setDirty(int row, int column, bool value){
    shelf.bins[xyToBin(row, column)].dirty=value;
  }

  std::string RobotData::xyToName(int row,int  column){
    if(row>=ROW_N || column>=COL_N){
      return "Nonexistant";
    }
    char r='A'+xyToBin(row,column);
    return std::string("")+r;
  }

  bool RobotData::isDirty(int row, int column){
    return shelf.bins[xyToBin(row, column)].dirty;
  }
  
  APC::OrderStatus RobotData::getWorkOrder(){
    return this->workOrder;
  }
  void RobotData::setWorkOrder(int row, int column,const std::string& itemName){
    std::cout << "Robotdata: received "+itemName+" into row" << row<<" column"<<column << "\n";
    APC::Order x(itemName);
    x.bin[0]=row;
    x.bin[1]=column;
    this->workOrder.push(x);
  }



  RobotData::~RobotData() {
    std::cout << "Your mom is being destructed\n";
  }

  RobotData::RobotData() 
  {
    std::cout << "Your mom is being constructed\n";
  };

  void RobotData::operator=(RobotData const&) {}; // Don't implement
  RobotData::RobotData(RobotData const&) {};              // Don't Implement

  void RobotData::setPhoto(int row, int column, const Camera::Image& frame){
    shelf.bins[xyToBin(row, column)].photo=frame;
  }
  Camera::Image RobotData::getFrame(int row, int column){
    return shelf.bins[xyToBin(row,column)].photo;
  }
}
