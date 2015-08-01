#include <cv.h>
#include <Camera/CameraModel.h>

namespace Camera{

  CameraModel::CameraModel(int w, int h, double fx, double fy, double s, double xc, double yc, double xCam, double yCam, double zCam, double aCam, double bCam, double gCam)
    :
      _K(3,3,CV_64F),
      _imWidth(w),
      _imHeight(h)
  {
    using cv::Mat;
    using cv::Mat_;
    _K.setTo(0);
    _K.at<double>(0,0)=fx;
    _K.at<double>(0,1)=s;
    _K.at<double>(0,2)=xc;
    _K.at<double>(1,1)=fy;
    _K.at<double>(1,2)=yc;
    _K.at<double>(2,2)=1;
    
    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<double>(4, 4) <<
              1,          0,           0, 0,
              0, cos(aCam), -sin(aCam), 0,
              0, sin(aCam),  cos(aCam), 0,
              0,          0,           0, 1);

    Mat RY = (Mat_<double>(4, 4) <<
              cos(bCam), 0, -sin(bCam), 0,
              0, 1,          0, 0,
              sin(bCam), 0,  cos(bCam), 0,
              0, 0,          0, 1);

    Mat RZ = (Mat_<double>(4, 4) <<
              cos(gCam), -sin(gCam), 0, 0,
              sin(gCam),  cos(gCam), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Translation 
    Mat T = (Mat_<double>(4,4) <<
              1, 0, 0, xCam,
              0, 1, 0, yCam,
              0, 0, 1, zCam,
              0, 0, 0, 1);
    _extr = T * RX * RY * RZ;

  }

    CameraModel::CameraModel(int w, int h, double fx, double fy, double s, double xc, double yc, const cv::Mat& extr_in)
    :
      _K(3,3,CV_64F),
      _extr(extr_in.clone()),
      _imWidth(w),
      _imHeight(h)
  {
    _K.setTo(0);
    _K.at<double>(0,0)=fx;
    _K.at<double>(0,1)=s;
    _K.at<double>(0,2)=xc;
    _K.at<double>(1,1)=fy;
    _K.at<double>(1,2)=yc;
    _K.at<double>(2,2)=1;
  }

  CameraModel::CameraModel(int w, int h, const cv::Mat& k_in, const cv::Mat& extr_in)
    :
      _imWidth(w),
      _imHeight(h),
      _extr(extr_in)
  {
    /** Only do the necessary checks */

    /** Size and type */
    assert(k_in.rows==3 && k_in.cols==3 && "Wrong-sized camera matrix");
    assert((k_in.depth()==CV_32F || k_in.depth()==CV_64F) && "Camera matrix must be a float or double matrix");

    if(k_in.depth()==CV_64F){
      _K=k_in.clone();
    }
    else{
      k_in.convertTo(_K, CV_64F);
    }

    /** Triangularity */
    assert(_K.at<double>(1,0)==0 && _K.at<double>(2,0)==0 && _K.at<double>(2,1)==0 && "Camera matrix is NOT upper-triangular");

    /** Focus length */
    double fx=_K.at<double>(0,0);
    double fy=_K.at<double>(1,1);
    assert(fx>0 && fy>0 && "Focus lengths cannot be negative");

  }

  cv::Mat CameraModel::getIntrinsic() const {
    return _K.clone();
  }

  CameraModel CameraModel::readFrom(const cv::FileNode& fs){

    /* Read YAML Vector */
    double fx, fy, s, xc, yc;
    int w, h;
    cv::Mat ext;

    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["s"] >> s;
    fs["xc"] >> xc;
    fs["yc"] >> yc;
    fs["width"] >> w;
    fs["height"] >> h;
    fs["extr"] >>  ext;

    assert(fx>0 && fy>0);

    return CameraModel(w, h, fx, fy, s, xc, yc, ext);
  }

  void CameraModel::writeTo(const std::string& name, cv::FileStorage& fs) const {

    /* Read YAML Vector */
    fs << "{";
    fs << "width" << _imWidth;
    fs << "height" << _imHeight;
    fs << "fx" << _K.at<double>(0,0);
    fs << "fy" << _K.at<double>(1,1);
    fs << "s" << _K.at<double>(0,1);
    fs << "xc" << _K.at<double>(0,2);
    fs << "yc" << _K.at<double>(1,2);
    fs << "extr" << _extr;
    fs << "}";
  }

  int CameraModel::getWidth() const {
    return _imWidth;
  }
  
  int CameraModel::getHeight() const {
    return _imHeight;
  }
  double CameraModel::getFx() const {
    return _K.at<double>(0,0);
  }
  double CameraModel::getFy() const {
    return _K.at<double>(1,1);
  }
  double CameraModel::getS() const {
    return _K.at<double>(0,1);
  }
  double CameraModel::getXc() const {
    return _K.at<double>(0,2);
  }
  double CameraModel::getYc() const {
    return _K.at<double>(1,2);
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
