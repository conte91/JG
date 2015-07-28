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
    private:
      std::string _id;
      std::string _myId;
      template<typename T>
        inline std::vector<T> readSequence(const cv::FileNode& n);
      std::shared_ptr<cv::linemod::Detector> _detector;
      std::shared_ptr<RendererIterator> _renderer_iterator;
      std::vector<cv::Mat> _Rmap;
      std::vector<cv::Vec3d> _Tmap;
      std::vector<float> _distMap;
      std::vector<cv::Mat> _Kmap, _hueHistMap;

    public:
      Model(const std::string& id, const boost::filesystem::path& myDir);

      /** Gets all the templates matching a certain template ID */
      const std::vector<cv::linemod::Template> getTemplates(int templateID) const;

      /** Gets all the templates of this model, for each template ID */
      const std::vector<cv::linemod::Template> getAllTemplates() const;
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
