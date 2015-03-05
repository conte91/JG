#pragma once

#include "ImageConsumer.h"
#include <string>

namespace Camera{

  class CameraView : public ImageConsumer
  {
    private:
      std::string _ID;
    public:
      CameraView(boost::shared_ptr<const ImageProvider> prov, const std::string& ID="CameraView");
      virtual ~CameraView()=0;
      virtual void Update();
  };

}
