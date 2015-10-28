#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

namespace cv {
namespace linemod {

using cv::FileNode;
using cv::FileStorage;
using cv::Mat;
using cv::Ptr;

/**
 * \brief Modality that computes quantized gradient orientations from a color image.
 */
class CV_EXPORTS ColorGradientFull : public Modality
{
public:
  /**
   * \brief Default constructor. Uses reasonable default parameter values.
   */
  ColorGradientFull();

  /**
   * \brief Constructor.
   *
   * \param weak_threshold   When quantizing, discard gradients with magnitude less than this.
   * \param num_features     How many features a template must contain.
   * \param strong_threshold Consider as candidate features only gradients whose norms are
   *                         larger than this.
   */
  ColorGradientFull(float weak_threshold, size_t num_features, float strong_threshold);

  virtual std::string name() const;

  virtual void read(const FileNode& fn);
  virtual void write(FileStorage& fs) const;

  float weak_threshold;
  size_t num_features;
  float strong_threshold;

protected:
  virtual Ptr<QuantizedPyramid> processImpl(const Mat& src,
                        const Mat& mask) const;
};

/**
 * \brief Factory function for detector using LINE-MOD algorithm with color gradients.
 *
 * Default parameter settings suitable for VGA images.
 * Using full object colour detection - not only on borders 
 */
CV_EXPORTS Ptr<Detector> getFullObjectLINEMOD();


} // namespace linemod
}
