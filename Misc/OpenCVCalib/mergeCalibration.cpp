#include <iostream>
#include <Camera/CameraModel.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <numeric>

int main(int argc, char** argv){
  using cv::FileStorage;
  using cv::FileNode;
  using std::accumulate;

  if(argc!=4){
    std::cerr << "Usage: " << argv[0] << " intrinsicName extrinsicName saveFile";
  }


  FileStorage fsInt( argv[1], FileStorage::READ );
  FileStorage fsExt( argv[2], FileStorage::READ );

  /** Reads the intrinsic parameters from the first file */
  auto cam1=Camera::CameraModel::readFrom(fsInt["camera_model"]);

  /** Reads the extrinsic parameters */
  cv::Mat mattia;
  fsExt["extrinsic_parameters"] >> mattia;

  std::vector <double> xs, ys, zs, as, bs, gs;
  for(int i=0; i<mattia.rows; ++i){
    xs.push_back(mattia.at<double>(i, 0));
    ys.push_back(mattia.at<double>(i, 1));
    zs.push_back(mattia.at<double>(i, 2));
    as.push_back(mattia.at<double>(i, 3));
    bs.push_back(mattia.at<double>(i, 4));
    gs.push_back(mattia.at<double>(i, 5));
  }

  double meanxs = accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
  double meanys = accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
  double meanzs = accumulate(zs.begin(), zs.end(), 0.0) / zs.size();
  double meanas = accumulate(as.begin(), as.end(), 0.0) / as.size();
  double meanbs = accumulate(bs.begin(), bs.end(), 0.0) / bs.size();
  double meangs = accumulate(gs.begin(), gs.end(), 0.0) / gs.size();

  Camera::CameraModel result(cam1.getWidth(), cam1.getHeight(), cam1.getFx(), cam1.getFy(), cam1.getS(), cam1.getXc(), cam1.getYc(), meanxs, meanys, meanzs, meanas, meanbs, meangs);

  FileStorage fsOut( argv[3], FileStorage::WRITE );
  fsOut << "camera_model" << result;

  return 0;
}
