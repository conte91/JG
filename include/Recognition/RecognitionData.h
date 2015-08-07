#pragma once
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <C5G/Pose.h>
#include <Camera/CameraModel.h>
#include "GiorgioUtils.h"
#include "Renderer3d.h"

#include <Img/ImageWMask.h>
#include "GLUTInit.h"
#include "Recognition.h"
#include "Model.h"

namespace Recognition{
  class RecognitionData{
    private:

      static C5G::Pose matrixToPose(cv::Mat m);
      typedef Camera::CameraModel CameraModel;
      typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

      GLUTInit _glutIniter;

      //Depth Camera Matrix
      const CameraModel _cameraModel;
      const double px_match_min_;

      const double th_obj_dist_; //"th_obj_dist", "Threshold on minimal distance between detected objects.", 0.04f);
      const double _threshold; //"threshold", "Matching threshold, as a percentage", 93.0f

      std::unordered_map<std::string, Model> _objectModels;

      std::string objsfolder_path;

      /** Pose estimation using PCL ICP 
       * @param pointsFromModel set of points of the ideal model (X,Y,Z)
       * @param pointsFromReference set of points from the reference scene (X,Y,Z)
       * @param finalTransformationMatrix output transformation matrix from ideal model to reference scene
       * @param resultPointClouds array of returned point clouds, in order: model point cloud, reference (i.e. scene) point cloud, aligned model point cloud 
       * @return true if ICP succeded, false otherwise
       */
      bool pclICP(const std::vector<cv::Vec3f>& pointsFromModel, const std::vector<cv::Vec3f>& pointsFromReference, Eigen::Matrix4d& finalTransformationMatrix, std::array< PCloud::Ptr , 3 >& resultPointClouds) const;

      bool updateGiorgio(const cv::Mat& const_rgb, const cv::Mat& depth_mm, const cv::Mat& filter_mask, 
      cv::Mat& Pose, const std::vector<std::string>& vect_objs_to_pick) const;

    public:
      C5G::Pose recognize(const Img::ImageWMask& frame, std::string what);

      /**
       * @param trainPath path to the trained models data
       * @param M camera model to use
       */
      RecognitionData(const std::string& trainPath, const CameraModel& m);

      PCloud::Ptr objectPointCloud(const std::string& objectID, const C5G::Pose& pose) const;
  };
}
