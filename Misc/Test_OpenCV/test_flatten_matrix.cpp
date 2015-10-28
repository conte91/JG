#include <cassert>
#include <cstdint>
#include <iostream>
#include <cv.h>

int main(){
  uint8_t myData[4][3][3]={{{1,2,3}, {4,5,6}, {7,8,9}},
    {{0,1,2}, {9,9,9}, {4,5,6}},
      {                    {4,5,6}, {1,2,3}, {0,9,7}},
      {{5,4,3}, {6,5,2}, {9,0,1}}};

  cv::Mat testMat(4,3,CV_8UC3, myData);
  std::cout << "size of testMat: " << testMat.rows << " x " << testMat.cols << std::endl;

  cv::Mat result=testMat.reshape(0,1).t();
  std::cout << " size of reshaped(1) testMat: " << result.rows << " x " << result.cols << std::endl;
  std::cout << "Reshaped to 1 row, 3 channels\n";
  std::cout << "1 matrix:\n\n" << result << "\n\n";
  result=result.reshape(1) ;/** Flatten the matrix to have 1 channel x N rows x 3 cols - exactly as we want it */
  std::cout << "Reshaped to N rows, 1 channel, 3 columns\n";
  std::cout << " size of original testMat: " << testMat.rows << " x " << testMat.cols << std::endl;
  std::cout << " size of reshaped testMat: " << result.rows << " x " << result.cols << std::endl;

  assert(result.type()==CV_8UC1);
  assert(result.cols==3);
  assert(result.rows==12);
  std::cout << "First matrix: \n " << testMat << "\n\n\n\nSecond:\n" << result << "\n\n";

  cv::Mat reresult=reresult.reshape(3, 3).t() ;/** Expand the matrix to 3channels x N rows x 1 col */
  std::cout << "Reshaped to N rows, 3 channel, 1 column & transposed\n";
  std::cout << " size of reshaped(1) result: " << reresult.rows << " x " << reresult.cols << std::endl;
  std::cout << "1 matrix:\n\n" << reresult << "\n\n";
  reresult=reresult.reshape(0,4);
  std::cout << "Reshaped to 3 channel, 1 row, N cols\n";
  std::cout << " size of original result: " << result.rows << " x " << result.cols << std::endl;
  std::cout << " size of reshaped result: " << reresult.rows << " x " << reresult.cols << std::endl;
  std::cout << "First matrix: \n " << result << "\n\n\n\nSecond:\n" << reresult << "\n\n";

  assert(reresult.type()==CV_8UC3);
  assert(reresult.cols==3);
  assert(reresult.rows==4);
  return 0;
}
