#include <Camera/DummyConsumer.h>
#include <Log/Log.h>

namespace Camera{
  DummyConsumer::DummyConsumer(boost::shared_ptr<const ImageProvider> provider, const std::string& ID)
  :ImageConsumer(provider, ID)
  {
    Log::out << "Inited dummy image consumer\n";
  }

  void DummyConsumer::update(){
    Log::out << "Updating...\n";
  }

  void setProvider(boost::shared_ptr<const ImageProvider> provider){
    Log::out << "Changed provider...\n";
  }
}
