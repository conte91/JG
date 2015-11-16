#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Recognition/Utils.h>
#include <Camera/CameraModel.h>
#include <Img/ImageWMask.h>

namespace Recognition{

Eigen::Affine3d tUpToOpenGLWorldTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& up){
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

Eigen::Affine3d tUpToCameraWorldTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& up){
    return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())*tUpToOpenGLWorldTransform(t,up);
}

Img::ImageWMask imageFromRender(const cv::Mat& rgb_in, const cv::Mat& depth_in, const cv::Mat& maskIn, const cv::Rect& rect_in, const Camera::CameraModel& cam){
  assert(rgb_in.type()==CV_8UC3);
  assert(maskIn.type()==CV_8UC1);
  cv::Mat rgb(cam.getHeight(), cam.getWidth(), rgb_in.type());
  cv::Mat depth(cam.getHeight(), cam.getWidth(), depth_in.type());
  cv::Mat mask(cam.getHeight(), cam.getWidth(), maskIn.type());
  rgb.setTo(cv::Scalar{0,0,0});
  depth.setTo(cv::Scalar{0});
  mask.setTo(cv::Scalar{0});

  rgb_in.copyTo(rgb(rect_in));
  depth_in.copyTo(depth(rect_in));
  maskIn.copyTo(mask(rect_in));

  return Img::ImageWMask{depth, rgb, mask};
}
}
