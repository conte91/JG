#pragma once
#include <opencv2/opencv.hpp>
#include <C5G/Pose.h>
#include <Camera/GiorgioUtils.h>
#include <Camera/Renderer3d.h>
#include <Camera/Recognition.h>

namespace Camera{
  class RecognitionData{
    private:

      static C5G::Pose matrixToPose(cv::Mat m);
      RecognitionData();
      //Depth Camera Matrix
      //TODO: Safier to read it from the calibration file
      const double dfx_;
      const double dfy_;
      const double dcx_;
      const double dcy_;
      const float px_match_min_ ;
      const float icp_dist_min_;
      cv::Mat K_depth_;

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
      static const std::string DEFAULT_OBJFOLDER_PATH;
      std::string objsfolder_path;
      /** LINE-MOD detector */
      cv::Ptr<cv::linemod::Detector> detector_ ;
      /** The renderer initialized with objects meshes, per object*/
      std::map<std::string, std::shared_ptr<RendererIterator> > renderer_iterators_;
      /** maps to cache the poses **/
      std::map<std::string,std::vector<float> > dist_map;
      std::map<std::string,std::vector<cv::Mat> > _hueHistMap;

      bool updateGiorgio(const cv::Mat& rgb, const cv::Mat& depth_meter, 
          cv::Ptr<cv::linemod::Detector>& detector_, std::map<std::string, std::shared_ptr<RendererIterator> >& renderer_iterators_, 
          std::map<std::string,std::vector<cv::Mat> >& Rs_ , std::map<std::string,std::vector<cv::Mat> >& Ts_, 
          std::map<std::string,std::vector<cv::Mat> >& Ks_ , std::map<std::string,std::vector<float> >& distances_,
          cv::Mat& Pose, const std::vector<std::string>& vect_objs_to_pick);
    public:
        static RecognitionData& getInstance();
        C5G::Pose recognize(const Image& frame, std::string what);
  };
}
