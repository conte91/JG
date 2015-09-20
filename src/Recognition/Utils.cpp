#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Recognition/Utils.h>

namespace Recognition{
Eigen::Affine3d tUpToWorldTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& up){
  /** Camera is at position T with up vector U, and is looking at the origin */
  /** We want to know the world transform wrt to the camera system
   */
  /** See https://www.opengl.org/sdk/docs/man2/xhtml/gluLookAt.xml */
  Eigen::Vector3d f=-t;
  Eigen::Vector3d up_n=up;
  f.normalize();
  up_n.normalize();
  Eigen::Vector3d s=f.cross(up);
  Eigen::Vector3d u=s.normalized().cross(f);
  Eigen::Matrix3d rotation;
  rotation << s[0],s[1],s[2],
              u[0],u[1],u[2],
              -f[0],-f[1],-f[2];

  Eigen::Affine3d result=Eigen::Affine3d::Identity();
  result*=rotation;
  result.translate(-t);

  return result;

}
}
