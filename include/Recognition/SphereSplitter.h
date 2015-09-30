#pragma once
#include <vector>
#include <array>
#include <Eigen/Core>
#include <functional>
#include <unordered_set>
namespace std{
  template<>
    struct hash<Eigen::Vector3d>{
     size_t operator()(const Eigen::Vector3d& x) const{
        return x[0]*1000000+x[1]*1000+x[2];
     }
    };
  template<>
    struct hash<Eigen::Vector2d>{
     size_t operator()(const Eigen::Vector2d& x) const{
        return x[0]*1000000+x[1];
     }
    };
}

namespace Recognition{
  class SphereSplitter
  {
    private:
      /** In spherical coordinates */
      struct uvSpherePointsEquals{
       bool operator()(const Eigen::Vector3d& x,const Eigen::Vector3d& y) const {
        return x.isApprox(y);
       }
      };
    public:
    /**
     * @param file_path the path of the mesh to render
     */
    SphereSplitter(unsigned int minimumNPoints);


    typedef std::unordered_set<Eigen::Vector3d, std::hash<Eigen::Vector3d>, uvSpherePointsEquals > UvPoints;

    /** Iterate to get to a different view
     * We don't implement the postfix operator on purpose
     * @return an incremented version of itself
     */
    const UvPoints& points() const ;

    private:
      typedef Eigen::Vector2d Vertex;
      typedef std::array<Vertex, 3> Face;
      UvPoints myPoints;

  };
}

