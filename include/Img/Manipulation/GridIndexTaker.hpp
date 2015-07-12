#include <boost/property_tree/ptree.hpp>

namespace Img{
  namespace Manipulation{
    template<typename T>
      std::vector<T> GridIndexTaker::readVector(const boost::property_tree::ptree& pt, const std::string& key){
        std::vector<T> r;
        for (auto& item : pt.get_child(key)){
          r.push_back(item.second.get_value<T>());
        }
        return r;

      }
  }
}

