#pragma once
#include <vector>
#include <array>
#include <Eigen/Core>
#include <functional>

namespace Recognition{
  class SphereSplitter
  {
    public:
      /**
       * @param file_path the path of the mesh to render
       */
    SphereSplitter(unsigned int minimumNPoints);

    /** Iterate to get to a different view
     * We don't implement the postfix operator on purpose
     * @return an incremented version of itself
     */
    const std::vector<Eigen::Vector3d>& points();

    private:
      /** In spherical coordinates */
      typedef Eigen::Vector2d Vertex;
      typedef std::array<Vertex, 3> Face;
      std::vector<Eigen::Vector3d> myPoints;

  };
}

namespace std{
  template<>
    struct hash<Eigen::Vector2d>{
     size_t operator()(const Eigen::Vector2d& x) const{
        return x[0]*1000000+x[1];
     }
    };
}
