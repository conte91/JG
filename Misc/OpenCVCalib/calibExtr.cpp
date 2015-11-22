#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/opencv.hpp>
#include <Camera/CameraModel.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/registration/icp.h>
#include <Img/Image.h>
#include <Recognition/Utils.h>

int main(int argc, char** argv){
  if(argc<8){
    std::cerr << "Usage: " << argv[0] << " square_size width height depth rgb camera_model output_file\n";
    return -1;
  }

  float square_size=::atof(argv[1]);
  int width=::atoi(argv[2]);
  int height=::atoi(argv[3]);

  cv::Mat depth=cv::imread(argv[4], CV_LOAD_IMAGE_ANYDEPTH|CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat rgb=cv::imread(argv[5], CV_LOAD_IMAGE_ANYDEPTH|CV_LOAD_IMAGE_COLOR);
  Img::Image img(depth, rgb);

  cv::FileStorage camFs(argv[6], cv::FileStorage::READ);
  Camera::CameraModel cam=Camera::CameraModel::readFrom(camFs["camera_model"]);
  camFs.release();

  cv::imshow("Image", img.rgb);
  while((cv::waitKey()&0xFF)!='q');

  std::vector<cv::Point2f> cornerPoints;
  if(!cv::findChessboardCorners(img.rgb, cv::Size(width, height), cornerPoints)){
    std::cerr << "Couldn't locate chessboard corners, sry\n";
    return -1;
  }

  cv::Mat imagePointsXYZ(depth.rows, depth.cols, CV_32FC3);
  cv::Mat drawnRGB=img.rgb.clone();

  const auto& K = cam.getIntrinsic();
  float fx = K(0, 0);
  float fy = K(1, 1);
  float s = K(0, 1);
  float cx = K(0, 2);
  float cy = K(1, 2);
  std::cout << "Intrinsic matrix: " << K << "\n";
  const cv::Mat& d=img.depth;
  for(int v=0; v<d.rows; ++v){
    for(int u=0; u<d.cols; ++u){
      /** Project back the point to XYZ space */
      {
        float z= d.at<float>(v,u);
        cv::Vec3f coordinates;

        coordinates[0] = (u - cx) / fx;

        if (s != 0){
          std::cout << "Axis distorsion not null!\n";
          coordinates[0] = coordinates[0] + (-(s / fy) * v + cy * s / fy) / fx;
        }

        coordinates[0] = coordinates[0]*z;
        coordinates[1] = (v - cy)*z * (1. / fy);
        coordinates[2] = z;
        imagePointsXYZ.at<cv::Vec3f>(v,u)=coordinates;
      }
    }
  }

  std::cout << "Chessboard corners' position: \n" ;
  for(auto& i: cornerPoints){
    std::cout << i << "\n";
  }
  std::cout << "\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr pChessBoard(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr refChessBoard(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer viewer("Chessboard");
  {
    int i=0;
    auto& pt0=cornerPoints[0];
    for(auto& pt : cornerPoints){
      /** Add point to our image for visualization purposes */
      int v(pt.y);
      int u(pt.x);

      /** Line to next point */
      cv::arrowedLine(drawnRGB, pt0, pt, cv::Scalar(0,0,0));
      pt0=pt;
      for(int k=-3; k<4; ++k){
        for(int j=-3; j<4; ++j){
          if(v+k>0 && u+j>0 && v+k < drawnRGB.rows && u+j<drawnRGB.cols){
            std::cout << "Drawing " << v+k << " " << u+j << "\n";
            drawnRGB.at<cv::Vec3b>(v+k,u+j)={0,255,0};
          }
          else{
            std::cout << "skip " << v+k << " " << u+j << "\n";
          }
        }
      }
      cv::Vec3f c=imagePointsXYZ.at<cv::Vec3f>(pt.y, pt.x);
      pcl::PointXYZ p, refP;
      p.x=c[0];
      p.y=c[1];
      p.z=c[2];
      /** Remember that points will be stored into cornerPoints into row-order, starting with the point closest to the top-left corner of the image. In our case, we want to align the system so that this is the point (0,yMax) and the opposite corner is (xMax, 0) - z points upwards */
      refP.y=square_size*(height-1-i/width);
      refP.x=square_size*(i%width);
      refP.z=0;
      pChessBoard->push_back(p);
      refChessBoard->push_back(refP);
      i++;
    }
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

  std::cout << "Computed extrinsics matrix: \n" << extrinsics.matrix() << "\n\n";
  std::cout << "Recomputing everything using the library algorithm..\n";
  auto e2=Recognition::extrinsicFromChessboard(square_size, width, height, img, cam);
  std::cout << "New matrix: " << e2.matrix() << "\n\n";
  if(!(e2.matrix().isApprox(extrinsics.matrix()))){
    std::cerr << "Wrong extrinsics implementation!\n";
    return -1;
  }
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorChessboard (pChessBoard, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorRefChessboard (refChessBoard, 255, 0, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligendIdealCB(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*refChessBoard, *aligendIdealCB, extrinsics);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorAlignedCB (aligendIdealCB, 0, 0, 255);
  viewer.addCoordinateSystem(0.1);
  viewer.addPointCloud<pcl::PointXYZ>(pChessBoard, colorChessboard, "chess");
  viewer.addPointCloud<pcl::PointXYZ>(refChessBoard, colorRefChessboard, "ref");
  viewer.addPointCloud<pcl::PointXYZ>(aligendIdealCB, colorAlignedCB, "aligned");
  cv::imshow("Sta cchiera", drawnRGB);
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
    cv::waitKey(100);
  }

  cv::Mat eMat;
  eigen2cv(extrinsics.matrix(), eMat);

  Camera::CameraModel updatedCam(cam.getWidth(), cam.getHeight(), cam.getIntrinsic(), eMat);
  cv::FileStorage outputFile(argv[7], cv::FileStorage::WRITE);
  outputFile << "camera_model" << updatedCam;
  std::cout << "Camera model saved to " << argv[7] << ".\n";
  return 0;
}
