#pragma once

namespace Camera{
  class Image{

    private:
      typedef cv::Mat Matrix;
      Matrix& _depth;
      Matrix& _rgb;

    public:
      Image(const cv::Mat& depth, const cv::Mat& rgb);
      ~Image();

      const Matrix& getDepthMap();
      const Matrix& getRGB();

  }
}
