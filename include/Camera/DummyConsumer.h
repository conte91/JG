#pragma once
#include "ImageConsumer.h"

namespace Camera{
  class DummyConsumer : public ImageConsumer {
    public:
      DummyConsumer(boost::shared_ptr<const ImageProvider> provider, const std::string& ID);
      virtual void update();

  };
}
