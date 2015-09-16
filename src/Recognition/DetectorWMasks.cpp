#include <Recognition/DetectorWMasks.h>

namespace cv{
  namespace linemod{
    static Rect cropTemplates(std::vector<Template>& templates)
    {
      int min_x = std::numeric_limits<int>::max();
      int min_y = std::numeric_limits<int>::max();
      int max_x = std::numeric_limits<int>::min();
      int max_y = std::numeric_limits<int>::min();

      // First pass: find min/max feature x,y over all pyramid levels and modalities
      for (int i = 0; i < (int)templates.size(); ++i)
      {
        Template& templ = templates[i];

        for (int j = 0; j < (int)templ.features.size(); ++j)
        {
          int x = templ.features[j].x << templ.pyramid_level;
          int y = templ.features[j].y << templ.pyramid_level;
          min_x = std::min(min_x, x);
          min_y = std::min(min_y, y);
          max_x = std::max(max_x, x);
          max_y = std::max(max_y, y);
        }
      }

      /// @todo Why require even min_x, min_y?
      if (min_x % 2 == 1) --min_x;
      if (min_y % 2 == 1) --min_y;

      // Second pass: set width/height and shift all feature positions
      for (int i = 0; i < (int)templates.size(); ++i)
      {
        Template& templ = templates[i];
        templ.width = (max_x - min_x) >> templ.pyramid_level;
        templ.height = (max_y - min_y) >> templ.pyramid_level;
        int offset_x = min_x >> templ.pyramid_level;
        int offset_y = min_y >> templ.pyramid_level;

        for (int j = 0; j < (int)templ.features.size(); ++j)
        {
          templ.features[j].x -= offset_x;
          templ.features[j].y -= offset_y;
        }
      }

      return Rect(min_x, min_y, max_x - min_x, max_y - min_y);
    }

    DetectorWMasks::DetectorWMasks()
      :
        Detector()
    {
    }

    DetectorWMasks::DetectorWMasks(const std::vector< Ptr<Modality> >& modalities, const std::vector<int>& T_pyramid)
      :
        Detector(modalities, T_pyramid)
    {
    }

    DetectorWMasks::DetectorWMasks(const Detector& src)
      :
        Detector(src)
    {
    }

    int DetectorWMasks::addTemplate(const std::vector<Mat>& sources, const std::string& class_id,
        const std::vector<Mat>& object_masks, Rect* bounding_box)
    {
      int num_modalities = static_cast<int>(modalities.size());
      std::vector<TemplatePyramid>& template_pyramids = class_templates[class_id];
      int template_id = static_cast<int>(template_pyramids.size());

      TemplatePyramid tp;
      tp.resize(num_modalities * pyramid_levels);

      assert(object_masks.size()==sources.size());
      // For each modality...
      for (int i = 0; i < num_modalities; ++i)
      {
        // Extract a template at each pyramid level
        Ptr<QuantizedPyramid> qp = modalities[i]->process(sources[i], object_masks[i]);
        for (int l = 0; l < pyramid_levels; ++l)
        {
          /// @todo Could do mask subsampling here instead of in pyrDown()
          if (l > 0)
            qp->pyrDown();

          bool success = qp->extractTemplate(tp[l*num_modalities + i]);
          if (!success)
            return -1;
        }
      }

      Rect bb = cropTemplates(tp);
      if (bounding_box)
        *bounding_box = bb;

      /// @todo Can probably avoid a copy of tp here with swap
      template_pyramids.push_back(tp);
      return template_id;
    }
  }
}
