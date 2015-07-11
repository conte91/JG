#include <iostream>
#include <cstdlib>
#include <Camera/Image.h>
#include <Camera/Cutter.h>
#include <Camera/ClickIndexTaker.h>
#include <Recognition/RecognitionData.h>
#include <Camera/ImageProvider.h>
#include <Camera/OpenniProvider.h>
#include <Camera/DummyProvider.h>
#include <Camera/FileProvider.h>
#include <Camera/OpenniWaitProvider.h>

int main(int argc, char** argv){

  using Camera::ImageProvider;

  if(argc!=3){
    std::cerr << "Specify a model path and a camera model (O,W,S,F,D) please!\n";
    return -1;

  }

  Recognition::RecognitionData::setModelPath(std::string(argv[1]));
  std::cout << "Welcome to the object recognition program!\n";

  std::cout << "Initializing camera..\n";

  std::unique_ptr<ImageProvider> camera([&argv] () -> ImageProvider*  { 
    try{
      switch(argv[2][0]){
        case 'W':
          return new Camera::OpenNIWaitProvider();
        case 'O':
          return new Camera::OpenNIProvider();
        case 'F':
          return new Camera::FileProvider();
        case 'D':
          return new Camera::DummyProvider();
        default:
          throw std::string("No valid provider model specified");
      }
    }
    catch(std::string s){
      std::cerr << "Error initializing camera: " << s << "\n";
      exit(-1);
    }
  } ()
  );
  std::cout << "Done.\n";

  bool haveFinished=false;
  while(!haveFinished){
    Recognition::RecognitionData& mySister=Recognition::RecognitionData::getInstance();

    std::string s;
    Camera::Image x=camera->getFrame();

    Camera::ClickIndexTaker taker(x);
    Camera::ImageWMask finalFrame(x, Camera::getMask(x, taker.getX1(), taker.getY1(), taker.getX2(), taker.getY2()));

    std::cout << "Which object do you want to recognize?\n";
    std::cin >> s;
    try{
      C5G::Pose result=mySister.recognize(finalFrame, s);
      std::cout << "I think the object is @pose " << result << "\n";
    }
    catch (std::string e){
      std::cout << "No, baybo, didn't get it. Maybe you could try again?\nDetails: " << e << "\n";
    }

    std::cout << "Do it again? (y/n) ";
    std::cin >> s;
    if(s=="n"){
      haveFinished=true;
    }

  }
  return 0;
}


