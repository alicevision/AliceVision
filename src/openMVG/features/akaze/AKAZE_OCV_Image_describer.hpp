#ifndef AKAZE_OCV_IMAGE_DESCRIBER_HPP
#define	AKAZE_OCV_IMAGE_DESCRIBER_HPP

/// Feature/Regions & Image describer interfaces
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

#include <cereal/cereal.hpp>

/// OpenCV Includes
#include <opencv2/opencv.hpp>

namespace openMVG {
namespace features {

///
//- Create an Image_describer interface that use an OpenCV feature extraction method
// i.e. with the AKAZE detector+descriptor
//--/!\ If you use a new Regions type you define and register it in
//   "openMVG/features/regions_factory.hpp" file.
///
// Reuse the existing AKAZE floating point Keypoint.
typedef features::AKAZE_Float_Regions AKAZE_OpenCV_Regions;
// Define the Interface
class AKAZE_OCV_Image_describer : public Image_describer
{
public:
  AKAZE_OCV_Image_describer():Image_describer(){}


  bool Set_configuration_preset(EDESCRIBER_PRESET preset)
  {
    return false;
  }
  /**
  @brief Detect regions on the image and compute their attributes (description)
  @param image Image.
  @param regions The detected regions and attributes (the caller must delete the allocated data)
  @param mask 8-bit gray image for keypoint filtering (optional).
     Non-zero values depict the region of interest.
  */
  bool Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = NULL)
  {
    cv::Mat img;
    cv::eigen2cv(image.GetMat(), img);

    std::vector< cv::KeyPoint > vec_keypoints;
    cv::Mat m_desc;

    cv::Ptr<cv::Feature2D> extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE);
    extractor->detectAndCompute(img, cv::Mat(), vec_keypoints, m_desc);

    if (!vec_keypoints.empty())
    {
      Allocate(regions);

      // Build alias to cached data
      AKAZE_OpenCV_Regions * regionsCasted = dynamic_cast<AKAZE_OpenCV_Regions*>(regions.get());
      // reserve some memory for faster keypoint saving
      regionsCasted->Features().reserve(vec_keypoints.size());
      regionsCasted->Descriptors().reserve(vec_keypoints.size());

      typedef Descriptor<float, 64> DescriptorT;
      DescriptorT descriptor;
      int cpt = 0;
      for(std::vector< cv::KeyPoint >::const_iterator i_keypoint = vec_keypoints.begin();
        i_keypoint != vec_keypoints.end(); ++i_keypoint, ++cpt){

        SIOPointFeature feat((*i_keypoint).pt.x, (*i_keypoint).pt.y, (*i_keypoint).size, (*i_keypoint).angle);
        regionsCasted->Features().push_back(feat);

        memcpy(descriptor.getData(),
               m_desc.ptr<typename DescriptorT::bin_type>(cpt),
               DescriptorT::static_size*sizeof(typename DescriptorT::bin_type));
        regionsCasted->Descriptors().push_back(descriptor);
      }
    }
    return true;
  };

  /// Allocate Regions type depending of the Image_describer
  void Allocate(std::unique_ptr<Regions> &regions) const
  {
    regions.reset( new AKAZE_OpenCV_Regions );
  }

  template<class Archive>
  void serialize( Archive & ar )
  {
  }
};

} // namespace features
} // namespace openMVG

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::AKAZE_OCV_Image_describer, "AKAZE_OCV_Image_describer");

#endif	/* AKAZE_OCV_IMAGE_DESCRIBER_HPP */

