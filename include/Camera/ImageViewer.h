#pragma once
#include "Image.h"

namespace Camera{
  class ImageViewer{
    public:
      void showImage(const Image& what);
      ImageViewer(const std::string& ID);
      void setTitle(const std::string& title);

    private:
      void updateMyself();
      std::string _title;
      Image _what;
      std::string _ID;
  };
}
