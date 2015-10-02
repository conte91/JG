#include <cassert>
#include <cv.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/rgbd.hpp>
#include <Camera/CameraModel.h>


namespace Camera{

  CameraModel::CameraModel(int w, int h, float fx, float fy, float s, float xc, float yc, float xCam, float yCam, float zCam, float aCam, float bCam, float gCam)
    :
      _K(3,3,CV_32F),
      _imWidth(w),
      _imHeight(h)
  {
    using cv::Mat;
    using cv::Mat_;
    _K=0;
    _K(0,0)=fx;
    _K(0,1)=s;
    _K(0,2)=xc;
    _K(1,1)=fy;
    _K(1,2)=yc;
    _K(2,2)=1;
    
    // Rotation matrices are saved as Rodriguez coefficients
    Mat RX = (Mat_<float>(4, 4) <<
              1,          0,           0, 0,
              0, cos(aCam), -sin(aCam), 0,
              0, sin(aCam),  cos(aCam), 0,
              0,          0,           0, 1);

    Mat RY = (Mat_<float>(4, 4) <<
              cos(bCam), 0, -sin(bCam), 0,
              0, 1,          0, 0,
              sin(bCam), 0,  cos(bCam), 0,
              0, 0,          0, 1);

    Mat RZ = (Mat_<float>(4, 4) <<
              cos(gCam), -sin(gCam), 0, 0,
              sin(gCam),  cos(gCam), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Translation 
    Mat T = (Mat_<float>(4,4) <<
              1, 0, 0, xCam,
              0, 1, 0, yCam,
              0, 0, 1, zCam,
              0, 0, 0, 1);
    _extr =cv::Mat( T * RX * RY * RZ);

  }

    CameraModel::CameraModel(int w, int h, float fx, float fy, float s, float xc, float yc, const cv::Mat& extr_in)
    :
      _K(3,3,CV_32F),
      _extr(extr_in.clone()),
      _imWidth(w),
      _imHeight(h)
  {
    _K=0;
    _K(0,0)=fx;
    _K(0,1)=s;
    _K(0,2)=xc;
    _K(1,1)=fy;
    _K(1,2)=yc;
    _K(2,2)=1;
    assert(extr_in.depth()==CV_32F);
  }

  CameraModel::CameraModel(int w, int h, const cv::Matx33f& k_in, const cv::Matx44f& extr_in)
    :
      _imWidth(w),
      _imHeight(h),
      _extr(extr_in),
      _K(k_in)
  {
    /** Only do the necessary checks */

    /** Size and type */
    assert(k_in.rows==3 && k_in.cols==3 && "Wrong-sized camera matrix");

    /** Triangularity */
    assert(_K(1,0)==0 && _K(2,0)==0 && _K(2,1)==0 && "Camera matrix is NOT upper-triangular");

    /** Focus length */
    float fx=_K(0,0);
    float fy=_K(1,1);
    assert(fx>0 && fy>0 && "Focus lengths cannot be negative");

  }

  cv::Matx33f CameraModel::getIntrinsic() const {
    return _K;
  }

  CameraModel CameraModel::readFrom(const cv::FileNode& fs){

    /* Read YAML Vector */
    float fx, fy, s, xc, yc;
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
    fs << "fx" << _K(0,0);
    fs << "fy" << _K(1,1);
    fs << "s" << _K(0,1);
    fs << "xc" << _K(0,2);
    fs << "yc" << _K(1,2);
    fs << "extr" << cv::Mat(_extr);
    fs << "}";
  }

  int CameraModel::getWidth() const {
    return _imWidth;
  }
  
  int CameraModel::getHeight() const {
    return _imHeight;
  }
  float CameraModel::getFx() const {
    return _K(0,0);
  }
  float CameraModel::getFy() const {
    return _K(1,1);
  }
  float CameraModel::getS() const {
    return _K(0,1);
  }
  float CameraModel::getXc() const {
    return _K(0,2);
  }
  float CameraModel::getYc() const {
    return _K(1,2);
  }

  Eigen::Affine3f CameraModel::getExtrinsic() const {
    Eigen::Matrix4f extrMatrix;
    cv2eigen(_extr, extrMatrix);
    Eigen::Affine3f result;
    result.matrix()=extrMatrix;
    return result;
  }

  //pcl::PointCloud<pcl::PointXYZ>::Ptr CameraModel::sceneToGlobalPointCloud(const Img::ImageWMask& _myData) const {
  //  return sceneToGlobalPointCloud(_myData, _myData.mask);
  //}

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CameraModel::sceneToCameraPointCloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Mat& m) const {
    cv::Mat mask;

    assert(depth.type()==CV_32FC1);
    assert(rgb.type()==CV_8UC3);
    if(!m.empty()){
      mask=m;
    }
    else{
      mask=cv::Mat(rgb.rows, rgb.cols, CV_8UC1);
      mask.setTo(cv::Scalar{255});
    }
    assert(rgb.size()==depth.size() && rgb.size()==mask.size() && "Images of different dimensions provided!");
    Eigen::Matrix<double, 6, Eigen::Dynamic> pointsUVDRGB(6,rgb.rows*rgb.cols);
    int countV=0;
    for(int v=0; v<rgb.rows; ++v){
      for(int u=0; u<rgb.cols; ++u){
        if(mask.at<uint8_t>(v,u)){
          const auto& pt=rgb.at<cv::Vec3b>(v,u);
          double d=depth.at<float>(v,u);
          if(d>0 && d <10 && d==d){
            pointsUVDRGB.col(countV) << d*u,d*v,d,pt[2],pt[1],pt[0];
            countV++;
          }
        }
      }
    }
    pointsUVDRGB.conservativeResize(Eigen::NoChange, countV+1);
    
    std::cout << "Resized\n";
    Eigen::Matrix<double, 6, 6> toMul=Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix3f k;
    Eigen::Matrix3d kInv;
    cv2eigen(_K, k);
    kInv=k.inverse().cast<double>();

    toMul.block<3,3>(0,0)=kInv;
    auto pointsXYZRGB=toMul*pointsUVDRGB;

    /** Eigen storage is row-major, so it is IMPORTANT to scan matrices in row order, otherwise cache misses -> HUNDREDS of times moar to execute (from 0.1s to >15min :| )*/
    Eigen::Matrix<double, Eigen::Dynamic, 6> pT=pointsXYZRGB.transpose();
    std::cout << "Mianonna\n";
    /** Fills cloud with points taken from (X,Y,Z) coordinates */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
    modelCloudPtr->resize(pointsXYZRGB.cols());
    for(int i=0; i<countV; ++i){
      auto vec=pT.row(i);
      pcl::PointXYZRGB pt{(uint8_t)vec[3],(uint8_t)vec[4],(uint8_t)vec[5]};
      pt.x=vec[0];
      pt.y=vec[1];
      pt.z=vec[2];
      modelCloudPtr->push_back(pt);
    }
    std::cout << "Tuanonna\n";
    return modelCloudPtr;
  }
  Eigen::Vector3d CameraModel::uvzToCameraFrame(double u, double v, double z) const{
    /** From the OpenCV_RGBD contrib project - depth_to_3d.cpp */
    float fx = _K(0, 0);
    float fy = _K(1, 1);
    float s = _K(0, 1);
    float cx = _K(0, 2);
    float cy = _K(1, 2);

    Eigen::Vector3d coordinates;

    coordinates[0] = (u - cx) / fx;

    if (s != 0){
      std::cout << "Ouch\n";
      coordinates[0] = coordinates[0] + (-(s / fy) * v + cy * s / fy) / fx;
    }

    coordinates[0] = coordinates[0]*z;
    coordinates[1] = (v - cy)*(z) * (1. / fy);
    coordinates[2] = z;
    return coordinates;
  }
  Eigen::Vector3d CameraModel::uvzToWorldFrame(double u, double v, double d) const{
    auto cPos=uvzToCameraFrame(u,v,d);
    Eigen::Vector4d cPos4{cPos[0], cPos[1], cPos[2], 1};

    Eigen::Matrix4f mat;
    cv2eigen(_extr, mat);
    return (mat.cast<double>()*cPos4).topRows<3>();
  }
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
