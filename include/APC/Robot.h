#pragma once
#include <iostream>
#include <boost/asio/ip/tcp.hpp>
#include <Img/Image.h>
#include <Camera/ImageProvider.h>
#include <C5G/C5G.h>
#include <C5G/Grasp.h>

namespace APC{
  class Robot : public C5G::C5G
  {
    private:
      Camera::ImageProvider::Ptr _provider;
    public:
      enum class CameraIndex
      {
        LEFT,
        RIGHT
      };
      void moveToBin(int row, int column);
      Robot(const std::string& ip, const std::string& sys_id, bool mustInit, const Camera::ImageProvider::Ptr& provider);
      Img::Image takePhoto(CameraIndex direction=CameraIndex::LEFT);
      void executeGrasp(const ::C5G::Grasp&);
  };
}

