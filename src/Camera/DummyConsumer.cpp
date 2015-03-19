#include <Camera/DummyConsumer.h>
#include <iostream>

namespace Camera{
  DummyConsumer::DummyConsumer(boost::shared_ptr<const ImageProvider> provider, const std::string& ID)
  :ImageConsumer(provider, ID)
  {
    std::cout << "Inited dummy image consumer\n";
  }

  void DummyConsumer::update(){
    std::cout << "Updating...\n";
  }

  void setProvider(boost::shared_ptr<const ImageProvider> provider){
    std::cout << "Changed provider...\n";
  }
}
