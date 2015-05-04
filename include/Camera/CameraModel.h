#pragma once
#include "Image.h"
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

class CameraModel {
  public:
    static const Pose cameraPose({ 2.20, 0, 0, 0, 0, 3.14159});


    pcl::PointXYZRGB pointToXYZ(const Image& frame, int x, int y);

  private:
    image_geometry::PinholeCameraModel _camModel;
};
