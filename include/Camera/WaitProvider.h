#pragma once

#include <opencv2/opencv.hpp>
#include <Img/Image.h>

namespace Camera{
  template<typename Provider>
    class WaitProvider : public Provider {
      public:
        WaitProvider(const std::string& ID="WaitProvider")
          :
          Provider(ID)
          {
          }
        virtual Img::Image getFrame() const {
          std::cout << 
            "==============================================="
            "||   Position the camera and press Enter..   ||"
            "==============================================="
            "\n";
          std::string s;
          std::cin >> s;
          return Provider::getFrame();

        }
    };
}

