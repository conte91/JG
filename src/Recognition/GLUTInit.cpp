#include <iostream>
#include <Recognition/GLUTInit.h>
#include <GL/glut.h>

namespace Recognition{
  bool GLUTInit::alreadyInited=false;

  GLUTInit::GLUTInit(){
    if(!alreadyInited){
      int argc=0;
      char** argv=nullptr;
      glutInit(&argc, argv);
      // By doing so, the window is not open
      glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_MULTISAMPLE);
      //glutInitDisplayMode(GLUT_DOUBLE);
      glutCreateWindow("Assimp renderer");
      // Initialize the environment
      glClearColor(0.f, 0.f, 0.f, 1.f);

      glEnable(GL_LIGHTING);
      glEnable(GL_LIGHT0); // Uses default lighting parameters

      glEnable(GL_DEPTH_TEST);
      GLint buf, sbuf;
      glGetIntegerv(GL_SAMPLE_BUFFERS, &buf);
      glGetIntegerv(GL_SAMPLES, &sbuf);
      std::cout << "Number of sample buffers is " << buf;
      std::cout << "Number of samples is " << sbuf;
      glEnable(GL_MULTISAMPLE);

      glEnable(GL_POLYGON_SMOOTH);
      glEnable(GL_LINE_SMOOTH);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


      glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
      glEnable(GL_NORMALIZE);
      GLfloat LightAmbient[]= {1.0f, 1.0f, 1.0f, 1.0f};
      GLfloat LightDiffuse[]= {0.0f, 0.0f, 0.0f, 1.0f};
      GLfloat LightPosition[]= { 0.0f, 0.0f, 1.0f, 0.0f };
      GLfloat LightAmbient2[]= {1.0f, 1.0f, 1.0f, 1.0f};
      GLfloat LightDiffuse2[]= {1.0f, 1.0f, 1.0f, 1.0f};
      GLfloat LightPosition2[]= { 0.0f, 0.0f, -1.0f, 0.0f };
      glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
      glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
      glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
      glEnable(GL_LIGHT0);
      //glLightfv(GL_LIGHT2, GL_AMBIENT, LightAmbient2);
      //glLightfv(GL_LIGHT2, GL_DIFFUSE, LightDiffuse2);
      //glLightfv(GL_LIGHT2, GL_POSITION, LightPosition2);
      //glEnable(GL_LIGHT2);
      alreadyInited=true;
    }
  }

}

