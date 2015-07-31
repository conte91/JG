#pragma once
#include "ImageConsumer.h"

namespace Camera{
  class DummyConsumer : public ImageConsumer {
    public:
      DummyConsumer(std::shared_ptr<const ImageProvider> provider, const std::string& ID="DummyConsumer");
      virtual void update();

  };
}
