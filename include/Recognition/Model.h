#pragma once
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <C5G/C5G.h>
#include <Recognition/db_linemod.h>
#include <Recognition/linemod_icp.h>
#include <Recognition/Renderer3d.h>
#include <Recognition/GiorgioUtils.h>
#include <Camera/CameraModel.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include "Mesh.h"
#include "Renderer3d.h"

namespace Recognition{
  class Model{
    public:
      /** Data associated with each template */
      struct TrainingData{
        cv::Mat R;
        cv::Vec3d T;
        double dist;
        Camera::CameraModel cam;
        cv::Mat hueHist;
      };

    private:
      std::string _myId;

      template<typename T>
        inline std::vector<T> readSequence(const cv::FileNode& n);
      cv::Ptr<cv::linemod::Detector> _detector;
      std::shared_ptr<RendererIterator> _renderer_iterator;
      std::vector<TrainingData> _myData;

      cv::Matx33d camTUp2ObjRot(const cv::Vec3d& tDir, const cv::Vec3d& upDir);
      int renderer_width;
      int renderer_height;
      double renderer_near;
      double renderer_far;
      double renderer_focal_length_x;
      double renderer_focal_length_y;
      int renderer_n_points;
      std::string mesh_file_path;
    public:

      /** Builds an ICP model from a training path */
      Model(const std::string& id, const boost::filesystem::path& myDir);

      /** Builds an empty ICP model */
      Model(const std::string& id, const std::string& meshFile, const Camera::CameraModel& cam);

      /** Gets all the templates matching a certain template ID */
      const std::vector<cv::linemod::Template> getTemplates(int templateID) const;

      /** Gets all the templates of this model, for each template ID */
      const std::vector<cv::linemod::Template> getAllTemplates() const;

      /** Loads this model from YML data taken from a folder */
      void readFrom(const std::string& id, const boost::filesystem::path& myDir);

      void saveToDirectory(const boost::filesystem::path& saveDir) const;

      void render(cv::Vec3d T, cv::Vec3d up, cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out) const;
      void renderImageOnly(cv::Vec3d T, cv::Vec3d up, cv::Mat &image_out, cv::Rect &rect_out) const;
      void renderDepthOnly(cv::Vec3d T, cv::Vec3d up, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out) const;

      void addTraining(const cv::Vec3d& T, const cv::Vec3d& up, const Camera::CameraModel& cam);

      TrainingData getData(int templateID) const;
      cv::Mat getR(int templateID) const;
      cv::Vec3d getT(int templateID) const;
      double getDist(int templateID) const;
      cv::Mat getK(int templateID) const;
      cv::Mat getHueHist(int templateID) const;
      Camera::CameraModel getCam(int templateID) const;
      const std::shared_ptr<RendererIterator> getRenderer() const;
      int numTemplates() const;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(const C5G::Pose& pose) const;
      Camera::CameraModel _camModel;

  };
}

#include "Model.hpp"
