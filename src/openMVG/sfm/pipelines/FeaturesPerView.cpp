#include "FeaturesPerView.hpp"

namespace openMVG {
namespace sfm {
  
bool loadFeaturesPerView(FeaturesPerView& featuresPerView,
                      const SfM_Data& sfmData,
                      const std::string& storageDirectory,
                      features::EImageDescriberType imageDescriberType)
{
  C_Progress_display my_progress_bar( sfmData.GetViews().size(), std::cout, "\n- Features Loading -\n" );
  
  // Read for each view the corresponding features and store them as PointFeatures
  bool invalid = false;
  
#ifdef OPENMVG_USE_OPENMP
  #pragma omp parallel
#endif
  for (auto iter = sfmData.GetViews().begin();
    (iter != sfmData.GetViews().end()) && (!invalid); ++iter)
  {
    
#ifdef OPENMVG_USE_OPENMP
  #pragma omp single nowait
#endif
    {
      const std::string featFile = stlplus::create_filespec(storageDirectory, 
              std::to_string(iter->second->id_view), 
              EImageDescriberType_enumToString(imageDescriberType) + ".feat");

      std::unique_ptr<features::Regions> regionsPtr;
      createImageDescriber(imageDescriberType)->Allocate(regionsPtr);
      
      if (!regionsPtr->LoadFeatures(featFile))
      {
        OPENMVG_LOG_WARNING("Invalid feature file: " << featFile);
        
#ifdef OPENMVG_USE_OPENMP
  #pragma omp critical
#endif
        invalid = true;
      }
      
#ifdef OPENMVG_USE_OPENMP
      #pragma omp critical
#endif
      {
        // save loaded Features as PointFeature
        featuresPerView.addFeatures(iter->second.get()->id_view, regionsPtr->GetRegionsPositions());
        ++my_progress_bar;
      }
    }
  }
  return !invalid;
}
  
} // namespace sfm
} // namespace openMVG