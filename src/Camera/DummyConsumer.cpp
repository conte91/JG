#include <Camera/DummyConsumer.h>
#include <iostream>

namespace Camera{
  DummyConsumer::DummyConsumer(std::shared_ptr<const ImageProvider> provider, const std::string& ID)
  :ImageConsumer(provider, ID)
  {
    std::cout << "Inited dummy image consumer\n";
  }

  void DummyConsumer::update(){
    std::cout << "Updating image...\n";
  }

  void setProvider(std::shared_ptr<const ImageProvider> provider){
    std::cout << "Changed provider...\n";
  }
}
