#pragma once
#include <opencv2/objdetect/objdetect.hpp>

/** Function that normalizes a vector
 * @param x the x component of the vector
 * @param y the y component of the vector
 * @param z the z component of the vector
 */
template<typename T>
static void normalize_vector(T & x, T&y, T&z)
{
  T norm = hypot(hypot(x,y),z);
  x /= norm;
  y /= norm;
  z /= norm;
}

namespace Recognition{
/** We know camera's T and UP vectors in object's coordinates (i.e. X right, Y forward, Z up),
 * we want to have object's transformations wrt OpenGL Camera reference frame (i.e. X right, Y up, Z backward)
 * @param t Camera position in _object_ coordinates
 * @param up Camera UP vector in _object_ coordinates
 * @returns Object transformation wrt to fixed camera frame (camera is in the origin, looking through negative Z axis
 */
Eigen::Affine3d tUpToOpenGLWorldTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& up);
/** We know camera's T and UP vectors in object's coordinates (i.e. X right, Y forward, Z up),
 * we want to have object's transformations wrt OpenCV Camera reference frame (i.e. X right, Y down, Z forward)
 * @param t Camera position in _object_ coordinates
 * @param up Camera UP vector in _object_ coordinates
 * @returns Object transformation wrt to fixed camera frame (camera is in the origin, looking through positive Z axis
 */
Eigen::Affine3d tUpToCameraWorldTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& up);
}

template <class T>
static inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

namespace std{
  template<>
    struct hash<cv::linemod::Match>{
     std::size_t operator()(const cv::linemod::Match& x) const{
       size_t result=0;
       hash_combine(result, x.x);
       hash_combine(result, x.y);
       hash_combine(result, x.template_id);
       hash_combine(result, x.class_id);
       return result;
     }
    };
}
