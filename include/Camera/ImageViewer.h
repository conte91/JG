#pragma once
#include <string>
#include <Img/Image.h>

namespace Camera{
  class ImageViewer{
    private:
      typedef Img::Image Image;

    public:
      void showImage(const Image& what);
      ImageViewer(const std::string& ID);
      void setTitle(const std::string& title);

    private:
      void updateMyself();
      std::string _title;
      Image _what;
      Image _toRender;
      std::string _ID;
  };
}
