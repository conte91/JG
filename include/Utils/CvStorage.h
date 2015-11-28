#pragma once
#include<iterator>

#include "Eigen2CV.h"

namespace cv{
  template<typename Key, typename Value>
  void write( FileStorage& fs, const std::string& name, const std::pair<Key, Value>& x){
    fs << "{" << "first" << x.first << "second" << x.second << "}";
  }

  template<typename Key, typename Value>
  void read(const FileNode& node, std::pair<Key,Value>& x, const std::pair<Key,Value>&& default_value=std::pair<Key,Value>{} ) {
    if(node.empty()){
      x=std::pair<Key,Value>(default_value);
      return;
    }
    else{
      node["first"] >> x.first;
      node["second"] >> x.second;
    }
  }
  template<typename Key, typename Value>
  void write( FileStorage& fs, const std::string& name, const std::map<Key, Value>& data){
    fs << "[";
    for(const auto& x : data){
      fs << "{" << "key" << x.first << "value" << x.second << "}";
    }
    fs << "]";
  }

  template<typename Key, typename Value>
  void read(const FileNode& node, std::map<Key,Value>& x, const std::map<Key,Value>& default_value=std::map<Key,Value>{} ) {
    if(node.empty()){
      x=default_value;
      return;
    }
    else{
      for(const auto& data:node){
        Key k;
        data["key"] >> k;
        data["value"] >> x[k];
      }
    }
  }

  template<typename T, size_t N>
  void write( FileStorage& fs, const std::string& name, const std::array<T,N>& data){
    fs << "[";
    std::for_each(std::begin(data), std::end(data), [&fs](T data){ fs << data; }); 
    fs << "]";
  }

  template<typename T, size_t N>
  void read(const FileNode& node, std::array<T,N>& x, const std::array<T,N>& default_value=std::array<T,N>{}){
    if(node.empty()){
      x=default_value;
      return;
    }
    else{
      size_t i=0;
      for(const auto& data:node){
        data >> x[i++];
      }
    }
  }

}
