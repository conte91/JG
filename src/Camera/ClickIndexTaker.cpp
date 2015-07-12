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
    cv::waitKey(1);

    std::array<std::mutex, 2>  clicks;
    clicks[0].lock();
    clicks[1].lock();
    std::array<int, 2> xc, yc;

    int count=0;

    /*** WE NEED THIS TO CAST THE FUNCTION TO A POINTER (PASSING CONTEXT) **/
    struct parameterToCallback{
      std::array<std::mutex, 2>*  clicks;
      std::array<int, 2>* xc;
      std::array<int, 2>* yc;
      int *count;
    };
    parameterToCallback theParameter({&clicks, &xc, &yc, &count});

    cv::setMouseCallback("Choose two points..", 
        [] (int event, int x, int y, int flags, void* stafava) -> void {
          parameterToCallback* par=static_cast<parameterToCallback*> (stafava);
          auto clicks=par->clicks;
          auto xc=par->xc;
          auto yc=par->yc;
          auto count=par->count;

          if(event==CV_EVENT_LBUTTONDOWN){
            std::cout << "Click.\nCount="<<*count <<"\n";
            if((*count)<2){
              std::cout << "Unlocking mutex #" << *count << "..\n";
              (*clicks)[*count].unlock();
              (*xc)[*count]=x;
              (*yc)[*count]=y;
              (*count)++;
            }
          }
        }
        , &theParameter);

    std::cout << "Click for first point..\n";
    while(!clicks[0].try_lock()){
      cv::waitKey(100);
    }
    std::cout << "Click for second point..\n";
    while(!clicks[1].try_lock()){
      cv::waitKey(100);
    }
    std::cout << "Thanks. Points: (" << xc[0] << "," << yc[0] << ") and (" << xc[1] << "," << yc[1] << ")\n";
    _x1=xc[0];
    _x2=xc[1];
    _y1=yc[0];
    _y2=yc[1];
    cv::destroyWindow("Choose two points..");
  }
}
