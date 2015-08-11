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
  cv::Mat mazzi, mattia;
  fsExt["extrinsic_parameters"] >> mazzi;
  if(mazzi.type()!=CV_32FC1){
    assert(mazzi.type()==CV_64FC1);
    std::cout << "Converting matrix to float\n";
    mazzi.convertTo(mattia, CV_32F);
  }
  else{
    mattia=mazzi;
  }


  std::vector <float> xs, ys, zs, r0s, r1s, r2s;
  for(int i=0; i<mattia.rows; ++i){
    xs.push_back(mattia.at<float>(i, 3));
    ys.push_back(mattia.at<float>(i, 4));
    zs.push_back(mattia.at<float>(i, 5));
    r0s.push_back(mattia.at<float>(i, 0));
    r1s.push_back(mattia.at<float>(i, 1));
    r2s.push_back(mattia.at<float>(i, 2));
  }

  float meanxs = accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
  float meanys = accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
  float meanzs = accumulate(zs.begin(), zs.end(), 0.0) / zs.size();
  float meanr0s = accumulate(r0s.begin(), r0s.end(), 0.0) / r0s.size();
  float meanr1s = accumulate(r1s.begin(), r1s.end(), 0.0) / r1s.size();
  float meanr2s = accumulate(r2s.begin(), r2s.end(), 0.0) / r2s.size();

  /** Convert Rodrigues vector to alpha-beta-gamma notation */
  cv::Mat matRot(3,3,CV_32F), extrinsics(4,4,CV_32F);
  std::cout << "Mean x: " << meanxs << "\n";
  std::cout << "Mean y: " << meanys << "\n";
  std::cout << "Mean z: " << meanzs << "\n";
  std::cout << "Mean r0: " << meanr0s << "\n";
  std::cout << "Mean r1: " << meanr1s << "\n";
  std::cout << "Mean r2: " << meanr2s << "\n";
  cv::Vec3f rodriguesRot(meanr0s, meanr1s, meanr2s), meanPos(meanxs, meanys, meanzs);
  std::cout << "Mean Rodrigues coefficients: " << rodriguesRot << "\n";
  cv::Rodrigues(rodriguesRot, matRot);
  std::cout << "Rotation matrix: " << matRot << "\n";
  std::cout << "Translation of origin from camera: " << meanPos << "\n";

  extrinsics.setTo(0);
  matRot.copyTo(extrinsics.rowRange(0,3).colRange(0,3));
  cv::Mat(meanPos).copyTo(extrinsics.rowRange(0,3).colRange(3,4));
  extrinsics.at<float>(3,3)=1;

  std::cout << "New extrinsics matrix: \n" << extrinsics << "\n";

  Camera::CameraModel result(cam1.getWidth(), cam1.getHeight(), cam1.getFx(), cam1.getFy(), cam1.getS(), cam1.getXc(), cam1.getYc(), extrinsics);

  FileStorage fsOut( argv[3], FileStorage::WRITE );
  fsOut << "camera_model" << result;

  return 0;
}
