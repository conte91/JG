#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

namespace cv{
  namespace linemod{
    class DetectorWMasks : public Detector
    {
      public:
        /**
         * \brief Empty constructor, initialize with read().
         */
      DetectorWMasks();

      /**
       * \brief Constructor.
       *
       * \param modalities       Modalities to use (color gradients, depth normals, ...).
       * \param T_pyramid        Value of the sampling step T at each pyramid level. The
       *                         number of pyramid levels is T_pyramid.size().
       */
      DetectorWMasks(const std::vector< Ptr<Modality> >& modalities, const std::vector<int>& T_pyramid);

      DetectorWMasks(const Detector& src);

      int addTemplate(const std::vector<Mat>& sources, const std::string& class_id,
          const std::vector<Mat>& object_masks, Rect* bounding_box = NULL);
    };
  }
}
