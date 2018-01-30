// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regionsIO.hpp"

#include <boost/progress.hpp>
#include <boost/filesystem.hpp>

#include <atomic>
#include <cassert>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfm {

std::unique_ptr<feature::Regions> loadRegions(const std::vector<std::string>& folders,
                                              IndexT viewId,
                                              const feature::ImageDescriber& imageDescriber)
{
  assert(!folders.empty());

  const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriber.getDescriberType());
  const std::string basename = std::to_string(viewId);

  std::string featFilename;
  std::string descFilename;

  for(const std::string& folder : folders)
  {
    const fs::path featPath = fs::path(folder) / std::string(basename + "." + imageDescriberTypeName + ".feat");
    const fs::path descPath = fs::path(folder) / std::string(basename + "." + imageDescriberTypeName + ".desc");

    if(fs::exists(featPath) && fs::exists(descPath))
    {
      featFilename = featPath.string();
      descFilename = descPath.string();
    }
  }

  if(featFilename.empty() || descFilename.empty())
    throw std::runtime_error("Can't find view " + basename + " region files");

  ALICEVISION_LOG_TRACE("Features filename: "    << featFilename);
  ALICEVISION_LOG_TRACE("Descriptors filename: " << descFilename);

  std::unique_ptr<feature::Regions> regionsPtr;
  imageDescriber.allocate(regionsPtr);

  try
  {
    regionsPtr->Load(featFilename, descFilename);
  }
  catch(const std::exception& e)
  {
    std::stringstream ss;
    ss << "Invalid " << imageDescriberTypeName << " regions files for the view " << basename << " : \n";
    ss << "\t- Features file : " << featFilename << "\n";
    ss << "\t- Descriptors file: " << descFilename << "\n";
    ss << "\t  " << e.what() << "\n";
    ALICEVISION_LOG_ERROR(ss.str());

    throw std::runtime_error(e.what());
  }

  ALICEVISION_LOG_TRACE("Region count: " << regionsPtr->RegionCount());
  return regionsPtr;
}

std::unique_ptr<feature::Regions> loadFeatures(const std::vector<std::string>& folders,
                                              IndexT viewId,
                                              const feature::ImageDescriber& imageDescriber)
{
  assert(!folders.empty());

  const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriber.getDescriberType());
  const std::string basename = std::to_string(viewId);

  std::string featFilename;

  for(const std::string& folder : folders)
  {
    const fs::path featPath = fs::path(folder) / std::string(basename + "." + imageDescriberTypeName + ".feat");
    if(fs::exists(featPath))
      featFilename = featPath.string();
  }

  if(featFilename.empty() )
    throw std::runtime_error("Can't find view " + basename + " features file");

  ALICEVISION_LOG_TRACE("Features filename: " << featFilename);

  std::unique_ptr<feature::Regions> regionsPtr;
  imageDescriber.allocate(regionsPtr);

  try
  {
    regionsPtr->LoadFeatures(featFilename);
  }
  catch(const std::exception& e)
  {
    std::stringstream ss;
    ss << "Invalid " << imageDescriberTypeName << " features file for the view " << basename << " : \n";
    ss << "\t- Features file : " << featFilename << "\n";
    ss << "\t  " << e.what() << "\n";
    ALICEVISION_LOG_ERROR(ss.str());

    throw std::runtime_error(e.what());
  }

  ALICEVISION_LOG_TRACE("Feature count: " << regionsPtr->RegionCount());
  return regionsPtr;
}

bool loadRegionsPerView(feature::RegionsPerView& regionsPerView,
            const SfMData& sfmData,
            const std::string& folder,
            const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
            const std::set<IndexT>& viewIdFilter)
{
  std::vector<std::string> featuresFolders = sfmData.getFeaturesFolders();
  featuresFolders.emplace_back(folder);

  boost::progress_display my_progress_bar( sfmData.GetViews().size() * imageDescriberTypes.size(), std::cout, "Loading regions\n");

  std::atomic_bool invalid(false);

  std::vector< std::unique_ptr<feature::ImageDescriber> > imageDescribers;
  imageDescribers.resize(imageDescriberTypes.size());

  for(std::size_t i =0; i < imageDescriberTypes.size(); ++i)
    imageDescribers[i] = createImageDescriber(imageDescriberTypes[i]);

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
         std::unique_ptr<feature::Regions> regionsPtr = loadRegions(featuresFolders, iter->second.get()->getViewId(), *imageDescribers[i]);

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
                      const std::string& folder,
                      const std::vector<feature::EImageDescriberType>& imageDescriberTypes)
{
  std::vector<std::string> featuresFolders = sfmData.getFeaturesFolders();
  featuresFolders.emplace_back(folder);

  boost::progress_display my_progress_bar( sfmData.GetViews().size(), std::cout, "Loading features\n" );

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
        std::unique_ptr<feature::Regions> regionsPtr = loadFeatures(featuresFolders, iter->second.get()->getViewId(), *imageDescribers[i]);

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
