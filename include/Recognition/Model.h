#pragma once
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <Recognition/db_linemod.h>
#include <Recognition/linemod_icp.h>
#include <Recognition/Renderer3d.h>
#include <Recognition/GiorgioUtils.h>

#include "Mesh.h"
#include "Renderer3d.h"

namespace Recognition{
  class Model{
    public:
      /** Data associated with each template */
      struct TrainingData{
        cv::Mat R;
        cv::Vec3d T;
        float dist;
        cv::Mat K;
        cv::Mat hueHist;
      };

    private:
      std::string _id;
      std::string _myId;
      template<typename T>
        inline std::vector<T> readSequence(const cv::FileNode& n);
      std::shared_ptr<cv::linemod::Detector> _detector;
      std::shared_ptr<RendererIterator> _renderer_iterator;
      std::vector<TrainingData> _myData;

    public:

      /** Builds an ICP model from a training path */
      Model(const std::string& id, const boost::filesystem::path& myDir);

      /** Builds an empty ICP model */
      Model(const std::string& id);

      /** Gets all the templates matching a certain template ID */
      const std::vector<cv::linemod::Template> getTemplates(int templateID) const;

      /** Gets all the templates of this model, for each template ID */
      const std::vector<cv::linemod::Template> getAllTemplates() const;

      void readFrom(const std::string& id, const boost::filesystem::path& myDir);
      TrainingData getData(int templateID) const;
      cv::Mat getR(int templateID) const;
      cv::Vec3d getT(int templateID) const;
      float getDist(int templateID) const;
      cv::Mat getK(int templateID) const;
      cv::Mat getHueHist(int templateID) const;
      const std::shared_ptr<RendererIterator> getRenderer() const;
      int numTemplates() const;

  };
}

#include "Model.hpp"
