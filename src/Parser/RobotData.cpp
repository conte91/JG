#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <Camera/Image.h>
#include <Parser/RobotData.h>

namespace InterProcessCommunication{

  std::ostream& operator<< (std::ostream& os, const InterProcessCommunication::RobotData& r){
    for(int i=0; i<12; ++i){
      os << "Bin " << (char) (i+'A') << ": ";
      for(int item=0; item<5; ++item){
        std::string s=r.shelf.bins[i].object[item];
        if(s!=""){
          os << r.shelf.bins[i].object[item];
          os << ",";
        }
      }
      os << "\n";
    }
  }

  int RobotData::getCurrentRow(){
    return _row;
  }

  int RobotData::getCurrentColumn(){
    return _column;
  }

  int RobotData::setCurrentRow(int row){
    _row=row;
  }

  int RobotData::setCurrentColumn(int column){
    _column=column;
  }

  RobotData& RobotData::getInstance() {
    static RobotData instance; // Guaranteed to be destroyed.
    return instance;
  }

  std::string RobotData::getBinItem(int row, int column, int item){
    return this->shelf.bins[(row*4)+column].object[item];
  }

  void RobotData::setBinItem(int row,int column,int item,const std::string& val){
    this->shelf.bins[(row*4)+column].object[item] = val;
  }

  int RobotData::xyToBin(int row, int column){
    return (row*4)+column;
  }

  void RobotData::setDirty(int row, int column){
    shelf.bins[xyToBin(row, column)].dirty;
  }

  bool RobotData::isDirty(int row, int column){
    return shelf.bins[xyToBin(row, column)].dirty;
  }
  
  std::vector<std::string> RobotData::getWorkOrder(){
    return this->workOrder;
  }
  void RobotData::setWorkOrder(int row, int column,const std::string& itemName){
    this->workOrder[(row*4)+column] = itemName;
  }

  //cv::Mat (const cv::Mat& orgImage)
  Camera::Image RobotData::getImageFrame(){
    using Camera::Image;
    Image frame;
    frame.rgb = cv::imread("a.jpg");
    frame.depth = cv::imread("a.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("rgb",cv::imread("a.jpg"));
    cv::imshow("bw",cv::imread("a.jpg",CV_LOAD_IMAGE_GRAYSCALE));
    return frame;
  }
  RobotData::RobotData() {};
  RobotData::~RobotData() {};
  void RobotData::operator=(RobotData const&) {}; // Don't implement
  RobotData::RobotData(RobotData const&) {};              // Don't Implement
}
