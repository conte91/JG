#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <iostream>

Eigen::MatrixXf imgToVectors(const cv::Mat& in){
  using cv::Mat;
  assert(in.type()==CV_8UC3);
    
  cv::Mat vectors(in.rows*in.cols, 3, CV_32FC1);
  for(size_t i=0; i<in.rows; ++i){
    for(size_t j=0; j<in.cols; ++j){
      auto& x=in.at<cv::Vec3b>(i,j);
      vectors.at<float>((i*in.cols+j),0)=x[0]-127.0f;
      vectors.at<float>((i*in.cols+j),1)=x[1]-127.0f;
      vectors.at<float>((i*in.cols+j),2)=x[2]-127.0f;
    }
  }

  Mat x=vectors;
  //assert(vectors.type()==CV_8UC1);
  //vectors.convertTo(x, CV_32F, 1, -127.0f);

  Eigen::MatrixXf vectorF;
  cv::cv2eigen(x, vectorF);
  return vectorF;
}
cv::Mat vectorsToImg(const Eigen::MatrixXf& vectorF, size_t rows, size_t cols){
  using cv::Mat;
  Mat vectors;
  cv::eigen2cv(vectorF, vectors);
  assert(vectors.type()==CV_32FC1);
  assert(vectors.cols==3);
  Mat result(rows, cols, CV_32FC3);
  for(size_t i=0; i<rows; ++i){
    for(size_t j=0; j<cols; ++j){
      result.at<cv::Vec3f>(i,j)[0]=vectors.at<float>(i*cols+j,0)+127.0f;
      result.at<cv::Vec3f>(i,j)[1]=vectors.at<float>(i*cols+j,1)+127.0f;
      result.at<cv::Vec3f>(i,j)[2]=vectors.at<float>(i*cols+j,2)+127.0f;
    }
  }
  Mat resultCast;
  result.convertTo(resultCast, CV_8U);
  assert(resultCast.type()==CV_8UC3);
  return resultCast;
}


/** From http://stattrek.com/matrix-algebra/covariance-matrix.aspx - Star Trek B) */
Eigen::Matrix3f eigenVectorsOfImage(const cv::Mat& const_input, const cv::Mat& mask=cv::Mat{}){
  using cv::Mat;
  using Eigen::MatrixXf;
  using Eigen::EigenSolver;
  assert(const_input.type()==CV_8UC3);
  cv::Mat input;
  if(mask.empty()){
    input=const_input.clone();
  }
  else{
    input=cv::Mat(input.size(), CV_8UC3);
    input.setTo(cv::Scalar{0,0,0});
    const_input.copyTo(input, mask);
  }
  Eigen::MatrixXf vectorF=imgToVectors(input);
  size_t N=input.rows*input.cols;
  //auto uni=Eigen::MatrixXf::Ones(N,1);
  Eigen::MatrixXf mean=vectorF.colwise().sum();
  auto dist=vectorF-(mean*(1.0f/N)).replicate(N,1);
  Eigen::MatrixXf dist2=dist;
  Eigen::Matrix3f cov=dist.transpose()*dist*(1.0f/N);
  EigenSolver<Eigen::Matrix3f> solver(cov);

  std::cout << "Covariance matrix:\n" << cov << "\n";
  return solver.eigenvectors().real();
}

cv::Mat rotateWithCovariance(const cv::Mat& in, const Eigen::Matrix3f& eigenVectors){
  Eigen::MatrixXf points=imgToVectors(in);
  Eigen::MatrixXf rotated=eigenVectors*points.transpose();

  cv::Mat result=vectorsToImg(rotated.transpose(), in.rows, in.cols);
  assert(result.type()==CV_8UC3);
  return result;
}

int main(int argc, char** argv) {

  using cv::Mat;
  using cv::waitKey;

  assert(argc==2 && "Must provide ONLY image file :)");

  Mat image,dst;
  image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

  std::cout << "Computing eigen vectors..\n";
  auto ev=eigenVectorsOfImage(image);
  std::cout << "Done!\n";
  std::cout << "Eigenvectors: " << ev << "\n";

  cv::Mat result=rotateWithCovariance(image, ev);
  std::cout << "New eigen vectors:\n" << eigenVectorsOfImage(result) << "\n";
  //assert(eigenVectorsOfImage(result).isApprox(Eigen::MatrixXf::Identity(3,3)));
  cv::Mat result2=result.clone();
  cv::cvtColor(result, result2, CV_BGR2HSV);
  Mat m[3];
  cv::split(result2, m);
  m[2].setTo(cv::Scalar{120});

  cv::Mat result3;
  cv::merge(m,3,result2);
  cv::cvtColor(result2, result3, CV_HSV2BGR);

  imshow("Original image", image);
  imshow("Converted image", result);
  imshow("Normalized image", result3);
  while((cv::waitKey(0)&0xFF)!='q');
  return 0;
}

