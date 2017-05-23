#include "SIFT_popSIFT_describer.hpp"
#include <openMVG/logger.hpp>

namespace openMVG {
namespace features {

bool SIFT_popSIFT_ImageDescriber::Describe(const image::Image<unsigned char>& image,
                                      std::unique_ptr<Regions>& regions,
                                      const image::Image<unsigned char>* mask)
{
  popsift::cuda::device_prop_t deviceInfo;
  deviceInfo.set(_cudaPipeIndex, _cudaInfo_printFirstTime);
  if(_cudaInfo_printFirstTime)
  {
    deviceInfo.print();
    _cudaInfo_printFirstTime = false; // only the first time!
  }

  std::unique_ptr<SiftJob> job(_popSift->enqueue(image.Width(), image.Height(), &image(0,0)));
  std::unique_ptr<popsift::Features> popFeatures(job->get());

  Allocate(regions);

  // Build alias to cached data
  SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get());
  regionsCasted->Features().reserve(popFeatures->getDescriptorCount());
  regionsCasted->Descriptors().reserve(popFeatures->getDescriptorCount());

  OPENMVG_LOG_TRACE("PopSIFT features count: " << popFeatures->getFeatureCount() << ", descriptors count: " << popFeatures->getDescriptorCount() << std::endl);

  for(const auto& popFeat: *popFeatures)
  {
    for(int orientationIndex = 0; orientationIndex < popFeat.num_ori; ++orientationIndex)
    {
      const popsift::Descriptor* popDesc = popFeat.desc[orientationIndex];
      Descriptor<unsigned char, 128> desc;

      // convertSIFT<unsigned char>(*popDesc, desc);
      for (std::size_t k = 0; k < 128; ++k)
        desc[k] = static_cast<unsigned char>(popDesc->features[k]);

      regionsCasted->Features().emplace_back(
        popFeat.xpos,
        popFeat.ypos,
        popFeat.sigma,
        popFeat.orientation[orientationIndex]);

      regionsCasted->Descriptors().emplace_back(desc);
    }
  }

  OPENMVG_LOG_TRACE("openMVG PopSIFT feature count : " << regionsCasted->RegionCount() << std::endl);

  return true;
}

} // namespace features
} // namespace openMVG
