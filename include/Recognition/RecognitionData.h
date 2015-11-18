#pragma once
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <C5G/Pose.h>
#include <Camera/CameraModel.h>
#include "Renderer3d.h"

#include <Img/ImageWMask.h>
#include "GLUTInit.h"
#include "Recognition.h"
#include "Model.h"

namespace Recognition{
  class RecognitionData{
    public:
      struct Match{
        double matchScore;
        Eigen::Affine3d pose;
      };

      typedef std::map<std::string, std::vector<Match> > ObjectMatches;
    private:
      struct FirstPassFoundItems{
        cv::linemod::Match match;
        Eigen::Affine3d objPose;
        cv::Mat rgb;
        cv::Mat depth;
        cv::Mat mask;
        cv::Rect rect;
        double matchPercentage;
      };

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
      bool pclICP(const std::vector<cv::Vec3f>& pointsFromModel, const std::vector<cv::Vec3f>& pointsFromReference, Eigen::Matrix4f& finalTransformationMatrix, std::array< PCloud::Ptr , 3 >& resultPointClouds) const;

      bool updateGiorgio(const cv::Mat& const_rgb, const cv::Mat& depth_m, const cv::Mat& filter_mask, 
      ObjectMatches& result, const std::vector<std::string>& vect_objs_to_pick) const;

    public:
      RecognitionData::ObjectMatches recognize(const Img::ImageWMask& frame,const std::vector<std::string>& what);

      /**
       * @param trainPath path to the trained models data
       * @param M camera model to use
       */
      RecognitionData(const std::string& trainPath, const CameraModel& m);

      PCloud::ConstPtr objectPointCloud(const std::string& objectID) const;
      PCloud::ConstPtr objectPointCloud(const std::string& objectID, const Eigen::Affine3d& pose) const;

      const Model& getModel(const std::string& name) const;

      std::vector<FirstPassFoundItems> makeAFirstPassRecognition(const cv::Mat& const_rgb, const cv::Mat& depth_m, const cv::Mat& filter_mask, 
                                                                 const std::vector<std::string>& whatToSee) const ;

  };
}
