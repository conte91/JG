#include <Camera/CameraModel.h>
#include <pcl/point_types.h>
// Fill in XYZ
// gg
//
pcl::PointXYZRGB pointToXYZ(const Image& frame, int x, int y){

  pcl::PointXYZRGB pt ;
  // Use correct principal point from calibration
  float center_x = cam_model_.cx();
  float center_y = cam_model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / cam_model_.fx();
  float constant_y = unit_scaling / cam_model_.fy();
  pt.x = (u - center_x) * depth * constant_x;
  pt.y = (v - center_y) * depth * constant_y;
  pt.z = DepthTraits<T>::toMeters(depth);
  pt.r=
    return pt;
}
