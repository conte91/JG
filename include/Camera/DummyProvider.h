#pragma once
#include <Camera/ImageProvider.h>

namespace Camera{
  class DummyProvider : public ImageProvider {
    public:
      DummyProvider(const std::string& ID="DummyProvider");
      virtual Image getFrame() const;
  };
}
