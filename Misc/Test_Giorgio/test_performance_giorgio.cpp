#include <iostream>
#include <cassert>
#include <cstdlib>
#include <Img/Image.h>
#include <Recognition/RecognitionData.h>
#include <Recognition/Utils.h>
#include <Camera/FileProviderAuto.h>

int main(int argc, char** argv){

  using Camera::ImageProvider;

  if(argc<=7){
    std::cerr << "Usage: " << argv[0] << " /path/to/models rgbPrefix depthPrefix squareSize chessboardWidth chessboardHeight objectName .. objectName\n";
    return -1;
  }

  std::cout << "Testing the performance for a set of objects into a single scene.\n";

  std::cout << "Initializing camera..\n";

  std::string rgbstring(argv[2]), depthstring(argv[3]);

  cv::FileStorage cameraFile("./camera_data.yml", cv::FileStorage::READ);

  double squareSize=::atof(argv[4]);
  int cWidth=::atoi(argv[5]);
  int cHeight=::atof(argv[6]);

  std::string inName;
  std::vector<std::string> whatToTake;
  for(int i=7; i<argc; ++i){
    whatToTake.emplace_back(argv[i]);
  }
  
  /** From here we take the intrinsics only */
  Camera::CameraModel camModel=Camera::CameraModel::readFrom(cameraFile["camera_model"]);

  Recognition::RecognitionData mySister(std::string(argv[1]), Camera::CameraModel::readFrom(cameraFile["camera_model"]));
  for(size_t i=0; ; ++i){
    std::ostringstream rgbCurrent, depthCurrent;
    rgbCurrent << rgbstring << i << ".png";
    depthCurrent << depthstring << i << ".png";
    std::unique_ptr<Camera::FileProviderAuto> cam;
    try{
      cam=std::move(decltype(cam){new Camera::FileProviderAuto(rgbCurrent.str(), depthCurrent.str())});
    }
    catch(std::string e){
      break;
    }

    Img::Image x=cam->getFrame();
    /** Match on the whole image */
    Img::Image::Matrix mask(x.rgb.size(), CV_8UC1, cv::Scalar{255});

    Img::ImageWMask finalFrame(x, mask);
    try{
      auto result=mySister.recognize(finalFrame, whatToTake);
      Eigen::Affine3f extr;
      try{
        extr=Recognition::extrinsicFromChessboard(squareSize, cWidth, cHeight, finalFrame, camModel);
      }
      catch(std::runtime_error e){
        std::cout << i << ": " << "NOCORNERS" << "\n";
        continue;
      }
      for(auto& x: whatToTake){
        if(result[x].empty()){
          std::cout << x << ": " << "NOTFOUND" << "\n";
        }
        else{
            auto globalPose=extr.inverse().cast<double>()*result[x][0].pose;
            std::cout << x << ": \n" << globalPose.matrix() << "\n";
        }
      }
    }
    catch (std::string e){
      std::cout << "Didn't get it\n";
    }

  }
  std::cout << "Exited.\n";
  return 0;
}

