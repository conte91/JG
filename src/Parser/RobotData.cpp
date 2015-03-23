#include <boost/python.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <Camera/Image.h>
#include <Parser/RobotData.h>

namespace InterProcessCommunication{
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
  
  int RobotData::start() {
    return 0;
  }

  int RobotData::stop() {
    return 0;
  }

  RobotData::RobotData() {};
  RobotData::~RobotData() {};
  void RobotData::operator=(RobotData const&) {}; // Don't implement
  RobotData::RobotData(RobotData const&) {};              // Don't Implement
}

#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
BOOST_PYTHON_MODULE(apcRobot)
{
  namespace python = boost::python;
  {
    python::class_<InterProcessCommunication::RobotData, boost::noncopyable>("RobotData", python::no_init)
      .def("getInstance",&InterProcessCommunication::RobotData::getInstance,python::return_value_policy<python::reference_existing_object>() )
      .staticmethod("getInstance")
      .def("getBinItem",&InterProcessCommunication::RobotData::getBinItem)
      .def("setBinItem",&InterProcessCommunication::RobotData::setBinItem)
      .def("start",&InterProcessCommunication::RobotData::start)
      .def("stop",&InterProcessCommunication::RobotData::stop)
      .def("getImageFrame",&InterProcessCommunication::RobotData::getImageFrame)
      ;
  }
}
