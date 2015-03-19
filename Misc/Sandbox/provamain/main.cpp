#include <C5G/C5G.h>
#include "ScanBins.h"
#include <Camera/DummyConsumer.h>
#include <Camera/DummyProvider.h>

int main(){
  using C5G::C5G;
  C5G robot;

  boost::shared_ptr<Camera::ImageProvider> x(new Camera::DummyProvider());
  Camera::DummyConsumer img(x); 
  ScanBins(robot, img);
  return 0;

}
