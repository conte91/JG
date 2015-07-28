#include <cv.h>
#include <Camera/CameraModel.h>

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

  CameraModel CameraModel::readFrom(const cv::FileNode& fs){

    /* Read JSON Vector */
    float fx, fy, s, xc, yc;
    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["s"] >> s;
    fs["xc"] >> xc;
    fs["yc"] >> yc;

    assert(fx>0 && fy>0);

    return CameraModel(fx, fy, s, xc, yc);
  }

  void CameraModel::writeTo(const std::string& name, cv::FileStorage& fs) const {

    /* Read JSON Vector */
    fs << "{";
    fs << "fx" << _K.at<float>(0,0);
    fs << "fy" << _K.at<float>(1,1);
    fs << "s" << _K.at<float>(0,1);
    fs << "xc" << _K.at<float>(0,2);
    fs << "yc" << _K.at<float>(1,2);
    fs << "}";
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

namespace cv{

  void write( FileStorage& fs, const std::string& name, const Camera::CameraModel& model){
    model.writeTo(name, fs);
  }

  void read(const FileNode& node, Camera::CameraModel& x, const Camera::CameraModel& default_value){
      if(node.empty())
        x=default_value;
      else
        x=Camera::CameraModel::readFrom(node);
  }

}
