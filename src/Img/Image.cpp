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

    /** Convert depth to meters if needed*/
    assert(d.depth()==CV_16U || d.depth()==CV_32F && "Depth needs to be CV_16U (mm) or CV_64F (m)!");
    if(d.depth()==CV_16U){
      d.convertTo(depth, CV_32F);
      depth = depth * 0.001;
    }
    else{
      depth=d;
    }
    assert(depth.depth()==CV_32F && depth.channels() == 1);
  }

  Image::Image(){
  }

}
