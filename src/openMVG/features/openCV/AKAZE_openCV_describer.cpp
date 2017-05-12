#include "AKAZE_openCV_describer.hpp"

#include "openMVG/image/image.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace openMVG {
namespace features {

bool AKAZE_openCV_ImageDescriber::Describe(const image::Image<unsigned char>& image,
                                           std::unique_ptr<Regions> &regions,
                                           const image::Image<unsigned char> * mask)
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
    AKAZE_Float_Regions* regionsCasted = dynamic_cast<AKAZE_Float_Regions*>(regions.get());

    // Reserve some memory for faster keypoint saving
    regionsCasted->Features().reserve(vec_keypoints.size());
    regionsCasted->Descriptors().reserve(vec_keypoints.size());

    typedef Descriptor<float, 64> DescriptorT;
    DescriptorT descriptor;
    int cpt = 0;

    for(std::vector< cv::KeyPoint >::const_iterator i_keypoint = vec_keypoints.begin();
      i_keypoint != vec_keypoints.end(); ++i_keypoint, ++cpt)
    {
      SIOPointFeature feat((*i_keypoint).pt.x, (*i_keypoint).pt.y, (*i_keypoint).size, (*i_keypoint).angle);
      regionsCasted->Features().push_back(feat);

      memcpy(descriptor.getData(),
             m_desc.ptr<typename DescriptorT::bin_type>(cpt),
             DescriptorT::static_size*sizeof(DescriptorT::bin_type));

      regionsCasted->Descriptors().push_back(descriptor);
    }
  }

  return true;
}

} //namespace features
} //namespace openMVG
