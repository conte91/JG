#pragma once

namespace Recognition{
  class GLUTInit{
    private:
      static bool alreadyInited;
    public:
      static void init();
  };
}
