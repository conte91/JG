#include <cassert>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <highgui.h>
#include <APC/Shelf.h>
#include <Img/Manipulation/GridIndexTaker.h>
#include <Camera/ImageViewer.h>
#include <Camera/ImageProvider.h>
#include <Camera/OpenniProvider.h>
#include <Camera/DummyProvider.h>
#include <Camera/FileProvider.h>
#include <Camera/OpenniWaitProvider.h>

std::vector<unsigned int> XSelected, YSelected;

/** Two sides of each line. #Lines=(N+1 for rows)+(N+1 for columns)*/
constexpr int N_POINTS=2*(APC::Shelf::WIDTH + 1 + APC::Shelf::HEIGHT + 1 );
std::array<std::mutex, N_POINTS> mutexes;
std::array<int, N_POINTS> xc, yc;
int count=0;

void selectNPointsCallback (int event, int x, int y, int flags, void* stafava) {
  if(event==CV_EVENT_LBUTTONDOWN){
    std::cout <<"PLICK\n";
    if(count<N_POINTS){
      std::cout << "Unlocking mutex #" << count << "..\n";
      mutexes[count].unlock();
      xc[count]=x;
      yc[count]=y;
      count++;
    }
  }
}

void execSetup(const Img::Image& img){
  using Img::Image;
  using Camera::ImageProvider;
  using APC::Shelf;
  using Img::Manipulation::GridIndexTaker;

  Camera::ImageViewer viewer("Frame");
  viewer.showImage(img);

  /** Let the user click on the needed points */
  cv::namedWindow("Choose points..");
  cv::imshow("Choose points..", img.rgb);
  cv::waitKey(1);
  for(unsigned int i=0; i<N_POINTS; ++i){
    mutexes[i].lock();
  }
  cv::setMouseCallback("Choose points..", selectNPointsCallback, NULL);
  for(unsigned int i=0; i<2*(Shelf::HEIGHT+1); ++i){
      std::cout << "Select point corresponding to " + (i%2 ? std::string("left") : std::string("right")) + " border of ROW # " << (i/2) << ".." << std::endl;
      while(!mutexes[i].try_lock()){
        cv::waitKey(100);
      }
  }
  for(unsigned int i=0; i<2*(Shelf::WIDTH+1); ++i){
      std::cout << "Select point corresponding to " + (i%2 ? std::string("upper") : std::string("lower")) + " border of COLUMN # " << (i/2) << ".. " << std::endl;
      while(!mutexes[2*(Shelf::HEIGHT+1)+i].try_lock()){
        cv::waitKey(100);
      }
  }

  /** Compute the coefficients */
  std::vector<double> mH, qH, mV, qV;
  cv::destroyWindow("Choose points..");

  /** Rows */
  for(unsigned int i=0; i<(Shelf::HEIGHT+1); ++i){
    double x1,y1,x2,y2;
    x1=xc[i*2];
    y1=yc[i*2];
    x2=xc[i*2+1];
    y2=yc[i*2+1];
    mH.push_back((y2-y1)/(x2-x1));
    qH.push_back(y1-mH.back()*x1);
  }

  for(unsigned int j=0; j<(Shelf::WIDTH+1); ++j){
    double x1,y1,x2,y2;
    unsigned int i=j+(Shelf::HEIGHT+1);
    x1=xc[i*2];
    y1=yc[i*2];
    x2=xc[i*2+1];
    y2=yc[i*2+1];
    mV.push_back((y2-y1)/(x2-x1));
    qV.push_back(y1-mV.back()*x1);
  }

  assert(mH.size()==Shelf::HEIGHT+1 && mV.size()==Shelf::WIDTH+1 && qH.size()==Shelf::HEIGHT+1 && qV.size()==Shelf::WIDTH+1);

  std::cout << "Saving the data..\n";
  GridIndexTaker toSave(Shelf::WIDTH, Shelf::HEIGHT, mH, qH, mV, qV);
  toSave.toFile("shelf_grid.data");

  std::cout << "Done. Loading the data again..\n";
  GridIndexTaker loadedOne=GridIndexTaker::fromFile("shelf_grid.data");


  /** Colour the grid in a checker fashion */
  cv::Mat m = img.rgb.clone();
  for(unsigned int i=0; i<Shelf::HEIGHT; ++i){
    for(unsigned int j=0; j<Shelf::WIDTH; ++j){

      /** Take the indices of the rectangle to fill - THIS IS THE STEP TESTING THE WHOLE SISTEM!! */
      loadedOne.updateCoords(i,j);
      int x1=std::min(loadedOne.getX1(), loadedOne.getX2());
      int x2=std::max(loadedOne.getX1(), loadedOne.getX2());
      int y1=std::min(loadedOne.getY1(), loadedOne.getY2());
      int y2=std::max(loadedOne.getY1(), loadedOne.getY2());

      cv::Mat roi = m(cv::Rect(x1, y1, x2, y2));
      /** Build a white or black square to put on the image */
      cv::Mat color(roi.size(), CV_8UC3, ((i+j)%2 ? cv::Scalar(0, 0, 0) : cv::Scalar(255,255,255)));

      double alpha = 0.3;
      cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);

    }
  }

  cv::namedWindow("Loaded grid");
  cv::imshow("Loaded grid", m);
  cv::waitKey(1);
}

int main(int argc, char** argv){

  using Img::Image;
  using Camera::ImageProvider;
  using APC::Shelf;
  using Img::Manipulation::GridIndexTaker;

  if(argc!=2){
    std::cerr << "Specify a model path and a camera model (O,W,S,F,D) please!\n";
    return -1;
  }

  std::cout << "Welcome to the shelf grid setup program!\n";

  std::cout << "Initializing camera..\n";

  std::unique_ptr<ImageProvider> camera([&argv] () -> ImageProvider*  { 
      try{
      switch(argv[1][0]){
      case 'W':
      return new Camera::OpenNIWaitProvider();
      case 'O':
      return new Camera::OpenNIProvider();
      case 'F':
      return new Camera::FileProvider();
      case 'D':
      return new Camera::DummyProvider();
      default:
      throw std::runtime_error("No valid provider model specified");
      }
      }
      catch(std::string s){
      std::cerr << "Error initializing camera: " << s << "\n";
      exit(-1);
      }
      } ()
      );
  std::cout << "Done.\n";

  bool goOn=true;
  while(goOn){
    std::cout << "Taking a frame..\n";

    Image img=camera->getFrame();
    execSetup(img);
    std::cout << "Is the result satisfactory? (Y/n)";
    std::string s;
    std::cin >> s;
    if(s!="n" && s=="N"){
      goOn=false;
    }
  }
  std::cout << "Thank you for using this beautiful software!\n";

  return 0;
}
