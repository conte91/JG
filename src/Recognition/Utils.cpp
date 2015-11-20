#include <cmath>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Recognition/Utils.h>
#include <Camera/CameraModel.h>
#include <C5G/Pose.h>
#include <Img/ImageWMask.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace Recognition{

  Eigen::Affine3f extrinsicFromChessboard(double squareSize, int width, int height, const Img::Image& img, const Camera::CameraModel& cam){

    std::vector<cv::Point2f> cornerPoints;
    if(!cv::findChessboardCorners(img.rgb, cv::Size(width, height), cornerPoints)){
      throw std::runtime_error( "Couldn't locate chessboard corners, sry");

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pChessBoard(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr refChessBoard(new pcl::PointCloud<pcl::PointXYZ>);

    const cv::Mat& d=img.depth;
    int i=0;
    for(auto& pt : cornerPoints){
      /** Project back the point to XYZ space */
      int v{pt.y};
      int u{pt.x};
      float z= d.at<float>(v,u);

      Eigen::Vector3d foundPt=cam.uvzToCameraFrame(u,v,z);
      pcl::PointXYZ p, refP;
      p.x=foundPt[0];
      p.y=foundPt[1];
      p.z=foundPt[2];
      /** Remember that points will be stored into cornerPoints into row-order, starting with the point closest to the top-left corner of the image. In our case, we want to align the system so that this is the point (0,yMax) and the opposite corner is (xMax, 0) - z points upwards */
      refP.y=squareSize*(height-1-i/width);
      refP.x=squareSize*(i%width);
      refP.z=0;
      pChessBoard->push_back(p);
      refChessBoard->push_back(refP);
      i++;
    }
    assert(refChessBoard->size()==pChessBoard->size());
    Eigen::Affine3f extrinsics=Eigen::Affine3f::Identity();
    /** Computes the transformation between the reference point cloud and the as-seen-by-the-camera point cloud, i.e. the extrinsics matrix of the camera :) */
    {
      Eigen::Vector4f cIdeal, cCB;
      Eigen::Vector3f t;
      pcl::compute3DCentroid(*refChessBoard, cIdeal);
      pcl::compute3DCentroid(*pChessBoard, cCB);
      std::cout << "cIdeal: " << cIdeal << "\ncCB: " << cCB << "\n";

      /** Translate the two clouds so that the centroid is on the origin */
      Eigen::Affine3f cTIdeal=Eigen::Affine3f::Identity(), cTCB=Eigen::Affine3f::Identity();
      cTIdeal.translation() = -cIdeal.topRows<3>();
      cTCB.translation() = -cCB.topRows<3>();

      pcl::PointCloud<pcl::PointXYZ> centeredIdeal, centeredCB;
      pcl::transformPointCloud(*refChessBoard, centeredIdeal, cTIdeal);
      pcl::transformPointCloud(*pChessBoard, centeredCB, cTCB);

      /** Use SVD to find the rotation between the two centered clouds */
      Eigen::Matrix3f H=Eigen::Matrix3f::Zero();
      for(size_t i=0; i<refChessBoard->size(); ++i){
        Eigen::Vector3f pCB{(*pChessBoard)[i].x, (*pChessBoard)[i].y, (*pChessBoard)[i].z};
        Eigen::Vector3f pId{(*refChessBoard)[i].x, (*refChessBoard)[i].y, (*refChessBoard)[i].z};
        H+=(pId-cIdeal.topRows<3>())*((pCB-cCB.topRows<3>()).transpose());
      }
      Eigen::JacobiSVD<Eigen::Matrix3f> decomposition(H,Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3f R=decomposition.matrixV()*decomposition.matrixU().transpose();
      if(R.determinant()<0){
        /** Reflection case: Invert 3rd column */
        R.col(2)*=-1;
      }
      assert(R.determinant()>0);
      t = -R*cIdeal.topRows<3>()+cCB.topRows<3>();
      extrinsics.linear()=R;
      extrinsics.translation()=t;
    }
    return extrinsics;
          
  }

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

/**TODOTODOTODOTODOTODO TODO */
#if 0
  C5G::Pose matrixToPose(const Eigen::Affine3d& m){
    float theta1=atan2(m.at<float>(1,2),m.at<float>(2,2));
    float c2=hypot(m.at<float>(0,0),m.at<float>(0,1));
    float theta2=atan2(-m.at<float>(0,2),c2);
    float s1=sin(theta1);
    float c1=cos(theta1);
    float theta3=atan2(s1*m.at<float>(2,0)-c1*m.at<float>(1,1),c1*m.at<float>(1,1)-s1*m.at<float>(2,1));
    return C5G::Pose(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3), theta1, theta2, theta3);
  }
#endif
}
