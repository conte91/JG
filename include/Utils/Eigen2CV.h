#pragma once
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

namespace cv{
template<typename Scalar, int R, int C, int _Options, int _MaxRows, int _MaxCols>
  void read(const FileNode& node, Eigen::Matrix<Scalar, R, C, _Options, _MaxRows, _MaxCols>& data, const Eigen::Matrix<Scalar, R, C, _Options, _MaxRows, _MaxCols>& default_value=Eigen::Matrix<Scalar, R, C, _Options, _MaxRows, _MaxCols>()){
    cv::Mat c;
    node >> c;
    cv::cv2eigen(c, data);
  }

template<typename Scalar, int R, int C, int _Options, int _MaxRows, int _MaxCols>
  void write( FileStorage& fs, const std::string& name, const Eigen::Matrix<Scalar, R, C, _Options, _MaxRows, _MaxCols>& data){
    cv::Mat result;
    cv::eigen2cv(data, result);
    fs << result;
  }
}
