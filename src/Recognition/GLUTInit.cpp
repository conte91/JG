#include <Recognition/GLUTInit.h>
#include <GL/glut.h>

namespace Recognition{
  bool GLUTInit::alreadyInited=false;

  GLUTInit::GLUTInit(){
    if(!alreadyInited){
      int argc=0;
      char** argv=nullptr;
      glutInit(&argc, argv);
      alreadyInited=true;
    }
  }

}

