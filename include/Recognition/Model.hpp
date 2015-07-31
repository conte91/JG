namespace Recognition{
  template<typename T>
    inline std::vector<T> Model::readSequence(const cv::FileNode& n) {
      std::vector<T> myMap;

      /** Reads the contents of a node inside a std::map */
      assert(n.type() == cv::FileNode::SEQ && "Data are not a sequence!");
      auto it = n.begin();
      while (it != n.end())
      {
        T m;
        it >> m;
        myMap.push_back(m);
      }

      return myMap;
    }
}

