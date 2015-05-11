#pragma once

namespace Camera{
  class GLUTInit{
    private:
      static bool alreadyInited;
    public:
      static void init();
  };
}
