#pragma once

namespace Recognition{
  class GLUTInit{
    private:
      static bool alreadyInited;
    public:
      /** Performs one-time initialization of GLUT: simply include an instance of this class when you need to be sure GLUT is initialized before doing anythin */
      GLUTInit();
  };
}
