#include <iostream>
#include <mutex>
#include <highgui.h>
#include <Camera/ClickIndexTaker.h>

namespace Camera{
  ClickIndexTaker::ClickIndexTaker(const Image& data)
    :
      _data(data)
  {
    updateCoords(0,0);
  }

  void ClickIndexTaker::updateCoords(int row, int column){
    cv::namedWindow("Choose two points..", cv::WINDOW_AUTOSIZE);
    cv::imshow("Choose two points..", _data.rgb);
    cv::Mat sblargaba=cv::imread("/home/simo/kinectData/rgb.png");
    cv::namedWindow("mySister", cv::WINDOW_AUTOSIZE);;
    cv::imshow("mySister", sblargaba);
    cv::waitKey(1);

    std::array<std::mutex, 2>  clicks;
    clicks[0].lock();
    clicks[1].lock();
    std::array<int, 2> xc, yc;

    /*** WE NEED THIS TO CAST THE FUNCTION TO A POINTER (PASSING CONTEXT) **/
    struct parameterToCallback{
      std::array<std::mutex, 2>&  clicks;
      std::array<int, 2>& xc;
      std::array<int, 2>& yc;
    };
    parameterToCallback theParameter({clicks, xc, yc});

    cv::setMouseCallback("Choose two points..", 
        [] (int event, int x, int y, int flags, void* stafava) -> void {
          static int count=0;
          parameterToCallback* par=static_cast<parameterToCallback*> (stafava);
          auto& clicks=par->clicks;
          auto& xc=par->xc;
          auto& yc=par->yc;

          if(event==CV_EVENT_LBUTTONDOWN){
            std::cout << "Click.\n";
            if(count<2){
              clicks[count].unlock();
              xc[count]=x;
              yc[count]=y;
              count++;
            }
          }
        }
        , &theParameter);

    std::cout << "Click for first point..\n";
    while(1){
      cv::waitKey(100);
      clicks[0].try_lock();
    }
    std::cout << "Click for second point..\n";
    while(1){
      cv::waitKey(100);
      clicks[1].try_lock();
    }
    std::cout << "Thanks. Points: (" << xc[0] << "," << yc[0] << ") and (" << xc[1] << "," << yc[1] << ")\n";
    cv::destroyWindow("Choose two points..");
  }
}
