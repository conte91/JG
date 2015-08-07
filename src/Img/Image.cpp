#include <iostream>
#include <Img/Image.h>

namespace Img{

  const int Image::ALLOWED_WIDTH=640;
  const int Image::ALLOWED_HEIGHT=480;

  Image::Image(const Matrix& d, const Matrix& r)
    :
      rgb(r)
  {
    assert(d.rows==ALLOWED_HEIGHT && d.cols==ALLOWED_WIDTH && "Wrong dimension");
    assert(r.rows==ALLOWED_HEIGHT && r.cols==ALLOWED_WIDTH && "Wrong dimension");
    assert(r.depth()==CV_8U && r.channels() == 3);

    /** We will use meter images as CV_64F */
    assert((d.depth()==CV_16U || d.depth()==CV_32F || d.depth()==CV_64F) && "Depth needs to be CV_16U (mm) or CV_32/64F (m)!");
    if(d.depth()!=CV_64F){
      if(d.depth()==CV_16U){
        /** Depth is in mm - convert it! */
        std::cout << "Converting to MMMMM\n";
        d.convertTo(depth, CV_64F, 0.001);
      }
      else{
        d.convertTo(depth, CV_64F);
      }
    }
    else{
      depth=d;
    }

    assert(depth.depth()==CV_64F && depth.channels() == 1);
  }

  Image::Image(){
  }

}
