#include <Camera/GLUTInit.h>
#include <GL/glut.h>

namespace Camera{
  bool GLUTInit::alreadyInited=false;

  void GLUTInit::init(){
    if(!alreadyInited){
      int argc=0;
      char** argv=nullptr;
      glutInit(&argc, argv);
      alreadyInited=true;
    }
  }

}

