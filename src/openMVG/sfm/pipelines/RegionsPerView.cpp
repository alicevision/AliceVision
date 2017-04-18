#include "RegionsPerView.hpp"

namespace openMVG {
namespace sfm {
  
bool loadRegionsPerView(RegionsPerView& regionsPerView,
            const SfM_Data& sfmData,
            const std::string& storageDirectory,
            features::EImageDescriberType imageDescriberType,
            const std::set<IndexT>& filter)
{
  C_Progress_display my_progress_bar( sfmData.GetViews().size(), std::cout, "\n- Regions Loading -\n");

  const std::string imageDescriberTypeName = EImageDescriberType_enumToString(imageDescriberType);
  bool invalid = false;

#ifdef OPENMVG_USE_OPENMP
  #pragma omp parallel num_threads(3)
#endif

 for (auto iter = sfmData.GetViews().begin();
   iter != sfmData.GetViews().end() && !invalid; ++iter)
 {
   
#ifdef OPENMVG_USE_OPENMP
  #pragma omp single nowait
#endif
   {
     const std::string basename = std::to_string(iter->second.get()->id_view);

     if(filter.empty() || filter.find(iter->second.get()->id_view) != filter.end())
     {
       const std::string featFilename = stlplus::create_filespec(storageDirectory, basename, imageDescriberTypeName + ".feat");
       const std::string descFilename = stlplus::create_filespec(storageDirectory, basename, imageDescriberTypeName + ".desc");

       std::unique_ptr<features::Regions> regionsPtr;
       createImageDescriber(imageDescriberType)->Allocate(regionsPtr);
       
       if (!regionsPtr->Load(featFilename, descFilename))
       {
         const std::string imageName = stlplus::create_filespec(sfmData.s_root_path, basename);
         OPENMVG_LOG_WARNING("Invalid " << imageDescriberTypeName << " regions files for the view : " << imageName);
         
#ifdef OPENMVG_USE_OPENMP
  #pragma omp critical
#endif
         invalid = true;
       }
       
#ifdef OPENMVG_USE_OPENMP
  #pragma omp critical
#endif
       {
         regionsPerView.addRegions(iter->second.get()->id_view, std::move(regionsPtr));
         ++my_progress_bar;
       }
     }
   }
 }
 return !invalid;
}
  
} // namespace sfm
} // namespace openMVG