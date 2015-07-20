#include <cv.h>
#include <Camera/CameraModel.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace Camera{

  CameraModel::CameraModel(float fx, float fy, float s, float xc, float yc)
    :
      _K(3,3,CV_32F)
  {
    _K.setTo(0);
    _K.at<float>(0,0)=fx;
    _K.at<float>(0,1)=s;
    _K.at<float>(0,2)=xc;
    _K.at<float>(1,1)=fy;
    _K.at<float>(1,2)=yc;
    _K.at<float>(2,2)=1;
  }

  cv::Mat CameraModel::getIntrinsic() const {
    return _K.clone();
  }

  CameraModel CameraModel::fromFile(const std::string& filename){
    namespace pt=boost::property_tree;

    /* Read JSON Vector */
    pt::ptree tree;
    pt::read_json(filename, tree);
    pt::ptree camera=tree.get_child("cameraModel");

    float fx=camera.get<float>("fx");
    float fy=camera.get<float>("fy");
    float s=camera.get<float>("s");
    float xc=camera.get<float>("xc");
    float yc=camera.get<float>("yc");

    assert(fx>0 && fy>0);

    return CameraModel(fx, fy, s, xc, yc);
  }

  void CameraModel::toFile(const std::string& filename) const {
    namespace pt=boost::property_tree;

    /* Read JSON Vector */
    pt::ptree tree;
    pt::ptree camera;
    camera.put<float>("fx", _K.at<float>(0,0));
    camera.put<float>("fy", _K.at<float>(1,1));
    camera.put<float>("s", _K.at<float>(0,1));
    camera.put<float>("xc", _K.at<float>(0,2));
    camera.put<float>("yc", _K.at<float>(1,2));
    tree.add_child("camera", camera);

    pt::write_json(filename, tree);
  }
#if 0
  TODO complete me :)
    pcl::PointXYZRGB pointToXYZ(const Image& frame, int x, int y){

      depth=frame.depth.at<double>(x,y);

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
#endif

}
