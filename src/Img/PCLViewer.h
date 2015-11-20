#include <Img/PCLViewer.h>
#include <stdexcept>

namespace Img{

  PCLViewer::PCLViewer(const std::string& title)
    :
      _clouds("title")
  {

  }

  void PCLViewer::addSource(PCloud::Ptr src, const std::string& id, bool visualize=true){
    if(_clouds.find(id)!=_clouds.end()){
      throw std::runtime_error("Cloud is already present!");
    }
    _clouds[id]=src;
    if(visualize){
      _viewer.addPointCloud(_clouds[id]);
    }
  }

  void PCLViewer::removeSource(const std::string id){
}
