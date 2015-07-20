#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <C5G/Pose.h>
#include <Camera/CameraModel.h>
#include "GiorgioUtils.h"
#include "Renderer3d.h"

#include <Img/ImageWMask.h>
#include "Recognition.h"

namespace Recognition{
  class RecognitionData{
    private:

      static C5G::Pose matrixToPose(cv::Mat m);
      typedef Camera::CameraModel CameraModel;
      typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

      //Depth Camera Matrix
      const CameraModel _cameraModel;
      const float px_match_min_;
      const float icp_dist_min_;

      //Training Params
      const int renderer_n_points_ ;
      const int renderer_angle_step_ ;
      const float renderer_radius_min_ ;
      const float renderer_radius_max_ ;
      const float renderer_radius_step_ ;
      const int renderer_width_ ;
      const int renderer_height_ ;
      const float renderer_near_ ;
      const float renderer_far_ ;
      const float renderer_focal_length_x_ ;
      const float renderer_focal_length_y_ ;
      const float th_obj_dist_; //"th_obj_dist", "Threshold on minimal distance between detected objects.", 0.04f);
      /** Because a map to a struct was far too easy to implement*/
      std::map<std::string,std::vector<cv::Mat> > _Rmap;
      std::map<std::string,std::vector<cv::Mat> > _Tmap;
      std::map<std::string,std::vector<cv::Mat> > _Kmap;
      std::map<std::string,std::vector<float> > _distMap;
      const float _threshold; //"threshold", "Matching threshold, as a percentage", 93.0f
      std::string objsfolder_path;
      /** LINE-MOD detector */
      cv::Ptr<cv::linemod::Detector> detector_ ;
      /** The renderer initialized with objects meshes, per object*/
      std::map<std::string, std::shared_ptr<RendererIterator> > renderer_iterators_;
      /** maps to cache the poses **/
      std::map<std::string,std::vector<float> > dist_map;
      std::map<std::string,std::vector<cv::Mat> > _hueHistMap;

      /** Pose estimation using PCL ICP 
       * @param pointsFromModel set of points of the ideal model (X,Y,Z)
       * @param pointsFromReference set of points from the reference scene (X,Y,Z)
       * @param finalTransformationMatrix output transformation matrix from ideal model to reference scene
       * @param resultPointClouds array of returned point clouds, in order: model point cloud, reference (i.e. scene) point cloud, aligned model point cloud 
       * @return true if ICP succeded, false otherwise
       */
      bool pclICP(const std::vector<cv::Vec3f>& pointsFromModel, const std::vector<cv::Vec3f>& pointsFromReference, Eigen::Matrix4d& finalTransformationMatrix, std::array< PCloud::Ptr , 3 >& resultPointClouds) const;

      bool updateGiorgio(const cv::Mat& rgb, const cv::Mat& depth_meter, const cv::Mat& mask,
          cv::Ptr<cv::linemod::Detector>& detector_, std::map<std::string, std::shared_ptr<RendererIterator> >& renderer_iterators_, 
          std::map<std::string,std::vector<cv::Mat> >& Rs_ , std::map<std::string,std::vector<cv::Mat> >& Ts_, 
          std::map<std::string,std::vector<cv::Mat> >& Ks_ , std::map<std::string,std::vector<float> >& distances_,
          cv::Mat& Pose, const std::vector<std::string>& vect_objs_to_pick) const;
    public:
      C5G::Pose recognize(const Img::ImageWMask& frame, std::string what);
      /**
       * @param trainPath path to the trained models data
       * @param M camera model to use
       */
      RecognitionData(const std::string& trainPath, const CameraModel& m);
  };
}
