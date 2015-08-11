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

  pcl::PointCloud<pcl::PointXYZ>::Ptr CameraModel::sceneToGlobalPointCloud(const Img::ImageWMask& _myData) const {
    return sceneToGlobalPointCloud(_myData, _myData.mask);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr CameraModel::sceneToGlobalPointCloud(const Img::Image& _myData, const cv::Mat& mask) const {
    cv::Mat_<cv::Vec3f> pointsXYZ;
    cv::rgbd::depthTo3d(_myData.depth, _K, pointsXYZ, mask);

    /** Fills model and reference pointClouds with points taken from (X,Y,Z) coordinates */
    pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

    /** Model PointCloud*/
    int mySize = pointsXYZ.rows*pointsXYZ.cols;
    modelCloudPtr->resize(mySize);
    modelCloudPtr->height = 1;
    modelCloudPtr->is_dense = true;

    for(int i=0; i<pointsXYZ.rows; ++i)
    {
      for(int j=0; j<pointsXYZ.cols; ++j){
        auto& tia=pointsXYZ[i][j];
        cv::Vec4f pt{tia[0],tia[1],tia[2],1};
        cv::Vec4f result=(_extr.inv()*pt);
        modelCloudPtr->points[i*pointsXYZ.cols+j].x=result[0];
        modelCloudPtr->points[i*pointsXYZ.cols+j].y=result[1];
        modelCloudPtr->points[i*pointsXYZ.cols+j].z=result[2];
      }
    }
    return modelCloudPtr;
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
