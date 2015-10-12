#include <Img/Image.h>
#include <Camera/OpenniProvider.h>
#include <Camera/Openni1Provider.h>
#include <Camera/Openni2Provider.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  OpenNIProvider::OpenNIProvider(const std::string& ID, int preferredVersion)
    :
      ImageProvider(ID),
      /** Use a lambda to dynamically choose the OpenNIProvider version */
      _p( [preferredVersion] () -> ImageProvider* {
          if(preferredVersion==1){
            try{
              return new OpenNI1Provider();
            }
            catch(std::string s){
              std::cout << "Couldn't find any OpenNI v1 enabled device (" << s << "), trying with v2..\n";
              return new OpenNI2Provider();
            }
          }
          else if(preferredVersion==2){
            try{
              return new OpenNI2Provider();
            }
            catch(std::string){
              std::cout << "Couldn't find any OpenNI v2 enabled device, trying with v1..\n";
              return new OpenNI1Provider();
            }
          }
          else{
            throw std::string("No valid OpenNI version provided!");
          }
        } ()
       )
  {
  }

  Img::Image OpenNIProvider::getFrame() const {
    return _p->getFrame();
  }
}
