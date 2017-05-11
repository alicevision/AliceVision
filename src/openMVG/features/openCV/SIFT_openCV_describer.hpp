#pragma once
#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/features/image_describer.hpp"
#include "openMVG/features/regions_factory.hpp"

#include <cereal/cereal.hpp>

namespace openMVG {

namespace image {

template <typename T>
class Image;

} //namespace image

namespace features {

class SIFT_openCV_Params
{
public:

  /**
   * @brief Use a preset to control the number of detected regions
   * @param preset The preset configuration
   * @return True if configuration succeed.
   */
  bool Set_configuration_preset(EDESCRIBER_PRESET preset);

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(
      cereal::make_nvp("grid_size", gridSize),
      cereal::make_nvp("max_total_keypoints", maxTotalKeypoints),
      cereal::make_nvp("n_octave_layers", nOctaveLayers),
      cereal::make_nvp("contrast_threshold", contrastThreshold),
      cereal::make_nvp("edge_threshold", edgeThreshold),
      cereal::make_nvp("sigma", sigma));
      //cereal::make_nvp("root_sift", root_sift));
  }

  /// Parameters
  std::size_t gridSize = 4;
  std::size_t maxTotalKeypoints = 1000;
  int nOctaveLayers = 6;            //< default opencv value is 3
  double contrastThreshold = 0.04;  //< default opencv value is 0.04
  double edgeThreshold = 10;
  double sigma = 1.6;
  //bool rootSift = true;
};

/**
 * @brief Create an Image_describer interface that use an OpenCV feature extraction method
 * Regions is the same as classic SIFT : 128 unsigned char
 */
class SIFT_openCV_ImageDescriber : public Image_describer
{
public:

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::SIFT_OCV;
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param preset The preset configuration
   * @return True if configuration succeed.
   */
  bool Set_configuration_preset(EDESCRIBER_PRESET preset) override
  {
    return _params.Set_configuration_preset(preset);
  }

  /**
   * @brief Detect regions on the image and compute their attributes (description)
   * @param image Image.
   * @param regions The detected regions and attributes (the caller must delete the allocated data)
   * @param mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<unsigned char>& image,
                std::unique_ptr<Regions> &regions,
                const image::Image<unsigned char> * mask = NULL);

  /**
   * @brief Allocate Regions type depending of the Image_describer
   * @param regions
   */
  void Allocate(std::unique_ptr<Regions> &regions) const
  {
    regions.reset( new SIFT_Regions );
  }

  template<class Archive>
  void serialize(Archive & ar)
  {
    ar(cereal::make_nvp("params", _params));
  }

private:
  SIFT_openCV_Params _params;
};

} //namespace features
} //namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_openCV_ImageDescriber, "SIFT_OPENCV_Image_describer");
