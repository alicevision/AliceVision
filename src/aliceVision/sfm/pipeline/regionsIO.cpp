// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "regionsIO.hpp"
#include "dependencies/progress/progress.hpp"

#include <atomic>


namespace aliceVision {
namespace sfm {


std::unique_ptr<feature::Regions> loadRegions(const std::string& folder, IndexT id_view, const feature::ImageDescriber& imageDescriber)
{
  const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriber.getDescriberType());
  const std::string basename = std::to_string(id_view);

  const std::string featFilename = stlplus::create_filespec(folder, basename, imageDescriberTypeName + ".feat");
  const std::string descFilename = stlplus::create_filespec(folder, basename, imageDescriberTypeName + ".desc");
  ALICEVISION_LOG_TRACE("featFilename: " << featFilename);
  ALICEVISION_LOG_TRACE("descFilename: " << descFilename);

  std::unique_ptr<feature::Regions> regionsPtr;
  imageDescriber.Allocate(regionsPtr);

  if (!regionsPtr->Load(featFilename, descFilename))
  {
    std::stringstream ss;
    ss << "Invalid " << imageDescriberTypeName << " regions files for the view : " << basename << ", describer type: " << imageDescriberTypeName << "\n";
    ss << "See features and descriptors files: " << featFilename << "\n" << descFilename << "\n";
    throw std::runtime_error(ss.str());
  }
  ALICEVISION_LOG_TRACE("RegionCount: " << regionsPtr->RegionCount());
  return regionsPtr;
}

bool loadRegionsPerView(feature::RegionsPerView& regionsPerView,
            const SfMData& sfmData,
            const std::string& folder,
            const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
            const std::set<IndexT>& viewIdFilter)
{
  C_Progress_display my_progress_bar( sfmData.GetViews().size() * imageDescriberTypes.size(), std::cout, "\n- Regions Loading -\n");

  std::atomic_bool invalid(false);


  std::vector< std::unique_ptr<feature::ImageDescriber> > imageDescribers;
  imageDescribers.resize(imageDescriberTypes.size());

  for(std::size_t i =0; i < imageDescriberTypes.size(); ++i)
  {
    imageDescribers[i] = createImageDescriber(imageDescriberTypes[i]);
  }


#pragma omp parallel num_threads(3)
 for (auto iter = sfmData.GetViews().begin();
   iter != sfmData.GetViews().end() && !invalid; ++iter)
 {
  #pragma omp single nowait
   {
     for(std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
     {
       if(viewIdFilter.empty() || viewIdFilter.find(iter->second.get()->getViewId()) != viewIdFilter.end())
       {
         std::unique_ptr<feature::Regions> regionsPtr = loadRegions(folder, iter->second.get()->getViewId(), *imageDescribers[i]);

         if(regionsPtr)
         {
  #pragma omp critical
           {
             regionsPerView.addRegions(iter->second.get()->getViewId(), imageDescriberTypes[i], regionsPtr.release());
             ++my_progress_bar;
           }
         }
         else
         {
           invalid = true;
         }
       }
     }
   }
 }
 return !invalid;
}


bool loadFeaturesPerView(feature::FeaturesPerView& featuresPerView,
                      const SfMData& sfmData,
                      const std::string& storageDirectory,
                      const std::vector<feature::EImageDescriberType>& imageDescriberTypes)
{
  C_Progress_display my_progress_bar( sfmData.GetViews().size(), std::cout, "\n- Features Loading -\n" );

  // Read for each view the corresponding features and store them as PointFeatures
  std::atomic_bool invalid(false);

  std::vector< std::unique_ptr<feature::ImageDescriber> > imageDescribers;
  imageDescribers.resize(imageDescriberTypes.size());

  for(std::size_t i =0; i < imageDescriberTypes.size(); ++i)
  {
    imageDescribers[i] = createImageDescriber(imageDescriberTypes[i]);
  }

#pragma omp parallel
  for (auto iter = sfmData.GetViews().begin();
    (iter != sfmData.GetViews().end()) && (!invalid); ++iter)
  {
#pragma omp single nowait
    {
      for(std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
      {
        const std::string featFile = stlplus::create_filespec(storageDirectory,
                std::to_string(iter->second->getViewId()),
                EImageDescriberType_enumToString(imageDescriberTypes[i]) + ".feat");

        ALICEVISION_LOG_TRACE("featFilename: " << featFile);

        std::unique_ptr<feature::Regions> regionsPtr;
        imageDescribers[i]->Allocate(regionsPtr);

        if (!regionsPtr->LoadFeatures(featFile))
        {
          ALICEVISION_LOG_WARNING("Invalid feature file: " << featFile);
          invalid = true;
        }

#pragma omp critical
        {
          // save loaded Features as PointFeature
          featuresPerView.addFeatures(iter->second.get()->getViewId(), imageDescriberTypes[i], regionsPtr->GetRegionsPositions());
          ++my_progress_bar;
        }
      }
    }
  }
  return !invalid;
}

  
} // namespace sfm
} // namespace aliceVision
