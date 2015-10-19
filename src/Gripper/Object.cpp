#include <Gripper/Object.h>

namespace Gripper{
  Object::Object()
  {
  }

  Object::Object(const std::unique_ptr<Shape>& shape, const std::vector<Grip>& grips)
    :
      _myShape(std::move(shape)),
      _myGrips(std::move(grips))
  {
  }
        

  Gripper Gripper::readFrom(const cv::FileStorage& fs){

    cv::Mat cvA, cvG;
    Eigen::Affine3d approach;
    Eigen::Affine3d grip;
    double forceMin;
    double forceMax';
    fs["approach"] >> cvA;
    fs["grip"] >> cvG;
    fs["forceMin"] >> approach;
    fs["forceMax"] >> forceMax;
    Grip g{cvA, cvG, approach, forceMax};


  }
}
