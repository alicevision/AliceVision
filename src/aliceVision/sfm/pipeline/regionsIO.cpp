// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regionsIO.hpp"

#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include <boost/algorithm/string/join.hpp>

#include <atomic>
#include <cassert>
#include <filesystem>

namespace fs = std::filesystem;

namespace aliceVision {
namespace sfm {

using namespace sfmData;

std::unique_ptr<feature::Regions> loadRegions(const std::vector<std::string>& folders, IndexT viewId, const feature::ImageDescriber& imageDescriber)
{
    assert(!folders.empty());
    
    const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriber.getDescriberType());
    const std::string basename = std::to_string(viewId);

    std::string featFilename;
    std::string descFilename;

    for (const std::string& folder : folders)
    {
        const fs::path featPath = fs::path(folder) / std::string(basename + "." + imageDescriberTypeName + ".feat");
        const fs::path descPath = fs::path(folder) / std::string(basename + "." + imageDescriberTypeName + ".desc");

        if (utils::exists(featPath) && utils::exists(descPath))
        {
            featFilename = featPath.string();
            descFilename = descPath.string();
        }
    }

    if (featFilename.empty() || descFilename.empty())
    {
        const std::string foldersStr = boost::algorithm::join(folders, ", ");
        throw std::runtime_error("Can't find view " + basename + " region files in folders " + foldersStr);
    }

    ALICEVISION_LOG_TRACE("Features filename: " << featFilename);
    ALICEVISION_LOG_TRACE("Descriptors filename: " << descFilename);

    std::unique_ptr<feature::Regions> regionsPtr;
    imageDescriber.allocate(regionsPtr);

    try
    {
        regionsPtr->Load(featFilename, descFilename);
    }
    catch (const std::exception& e)
    {
        std::stringstream ss;
        ss << "Invalid " << imageDescriberTypeName << " regions files for the view " << basename << " : \n";
        ss << "\t- Features file : " << featFilename << "\n";
        ss << "\t- Descriptors file: " << descFilename << "\n";
        ss << "\t  " << e.what() << "\n";
        ALICEVISION_LOG_ERROR(ss.str());

        throw;
    }

    ALICEVISION_LOG_TRACE("Region count: " << regionsPtr->RegionCount());
    return regionsPtr;
}

std::unique_ptr<feature::Regions> loadFeatures(const std::vector<std::string>& folders, IndexT viewId, const feature::ImageDescriber& imageDescriber)
{
    assert(!folders.empty());

    const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriber.getDescriberType());
    const std::string basename = std::to_string(viewId);

    std::string featFilename;

    // build up a set with normalized paths to remove duplicates
    std::set<std::string> foldersSet;
    for (const auto& folder : folders)
    {
        if (utils::exists(folder))
        {
            foldersSet.insert(fs::canonical(folder).string());
        }
    }

    for (const auto& folder : foldersSet)
    {
        const fs::path featPath = fs::path(folder) / std::string(basename + "." + imageDescriberTypeName + ".feat");
        if (utils::exists(featPath))
            featFilename = featPath.string();
    }

    if (featFilename.empty())
    {
        const std::vector<std::string> folders(foldersSet.begin(), foldersSet.end());
        const std::string foldersStr = boost::algorithm::join(folders, ", ");
        throw std::runtime_error("Can't find view " + basename + " features files in folders " + foldersStr);
    }

    ALICEVISION_LOG_DEBUG("Features filename: " << featFilename);

    std::unique_ptr<feature::Regions> regionsPtr;
    imageDescriber.allocate(regionsPtr);

    try
    {
        regionsPtr->LoadFeatures(featFilename);
    }
    catch (const std::exception& e)
    {
        std::stringstream ss;
        ss << "Invalid " << imageDescriberTypeName << " features file for the view " << basename << " : \n";
        ss << "\t- Features file : " << featFilename << "\n";
        ss << "\t  " << e.what() << "\n";
        ALICEVISION_LOG_ERROR(ss.str());

        throw;
    }

    ALICEVISION_LOG_TRACE("Feature count: " << regionsPtr->RegionCount());
    return regionsPtr;
}

bool loadFeaturesPerDescPerView(std::vector<std::vector<std::unique_ptr<feature::Regions>>>& featuresPerDescPerView,
                                const std::vector<IndexT>& viewIds,
                                const std::vector<std::string>& folders,
                                const std::vector<feature::EImageDescriberType>& imageDescriberTypes)
{
    if (folders.empty())
    {
        ALICEVISION_LOG_ERROR("Cannot load features, no folders provided");
        return false;
    }
    if (viewIds.empty())
    {
        ALICEVISION_LOG_ERROR("Cannot load features, no view ids provided");
        return false;
    }
    if (imageDescriberTypes.empty())
    {
        ALICEVISION_LOG_ERROR("Cannot load features, no image desciber types provided");
        return false;
    }

    std::vector<std::unique_ptr<feature::ImageDescriber>> imageDescribers;
    imageDescribers.resize(imageDescriberTypes.size());

    for (std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
        imageDescribers.at(i) = createImageDescriber(imageDescriberTypes.at(i));

    featuresPerDescPerView.resize(imageDescribers.size());

    std::atomic_bool loadingSuccess(true);

    for (int descIdx = 0; descIdx < imageDescribers.size(); ++descIdx)
    {
        std::vector<std::unique_ptr<feature::Regions>>& featuresPerView = featuresPerDescPerView.at(descIdx);
        featuresPerView.resize(viewIds.size());

#pragma omp parallel for
        for (int viewIdx = 0; viewIdx < viewIds.size(); ++viewIdx)
        {
            try
            {
                featuresPerView.at(viewIdx) = loadFeatures(folders, viewIds.at(viewIdx), *imageDescribers.at(descIdx));
            }
            catch (const std::exception& e)
            {
                const feature::EImageDescriberType d = imageDescribers.at(descIdx)->getDescriberType();

#pragma omp critical
                {
                    ALICEVISION_LOG_ERROR("Cannot load features for View " << viewIds.at(viewIdx) << " for describer type " << feature::EImageDescriberType_enumToString(d) << ".");
                    ALICEVISION_LOG_ERROR(e.what());
                }
                loadingSuccess = false;
            }
        }
    }

    return loadingSuccess;
}

bool loadRegionsPerView(feature::RegionsPerView& regionsPerView,
                        const SfMData& sfmData,
                        const std::vector<std::string>& folders,
                        const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
                        const std::set<IndexT>& viewIdFilter)
{
    std::vector<std::string> featuresFolders = sfmData.getFeaturesFolders();        // add sfm features folders
    featuresFolders.insert(featuresFolders.end(), folders.begin(), folders.end());  // add user features folders
    auto last = std::unique(featuresFolders.begin(), featuresFolders.end());
    featuresFolders.erase(last, featuresFolders.end());

    auto progressDisplay =
      system::createConsoleProgressDisplay(sfmData.getViews().size() * imageDescriberTypes.size(), std::cout, "Loading regions\n");

    std::atomic_bool invalid(false);

    std::vector<std::unique_ptr<feature::ImageDescriber>> imageDescribers;
    imageDescribers.resize(imageDescriberTypes.size());

    for (std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
        imageDescribers.at(i) = createImageDescriber(imageDescriberTypes.at(i));

#pragma omp parallel num_threads(3)
    for (auto iter = sfmData.getViews().begin(); iter != sfmData.getViews().end() && !invalid; ++iter)
    {
#pragma omp single nowait
        {
            for (std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
            {
                if (viewIdFilter.empty() || viewIdFilter.find(iter->second.get()->getViewId()) != viewIdFilter.end())
                {
                    std::unique_ptr<feature::Regions> regionsPtr;
                    try
                    {
                        regionsPtr = loadRegions(featuresFolders, iter->second.get()->getViewId(), *(imageDescribers.at(i)));                        
                    }
                    catch (const std::exception& e)
                    {
                        invalid = true;
#pragma omp critical
                        {
                            ALICEVISION_LOG_ERROR(e.what());
                        }
                        continue;
                    }
#pragma omp critical
                    {
                        regionsPerView.addRegions(iter->second.get()->getViewId(), imageDescriberTypes.at(i), regionsPtr.release());
                        ++progressDisplay;
                    }
                }
            }
        }
    }
    return !invalid;
}

bool loadFeaturesPerView(feature::FeaturesPerView& featuresPerView,
                         const SfMData& sfmData,
                         const std::vector<std::string>& folders,
                         const std::vector<feature::EImageDescriberType>& imageDescriberTypes)
{
    std::vector<std::string> featuresFolders = sfmData.getFeaturesFolders();        // add sfm features folders
    featuresFolders.insert(featuresFolders.end(), folders.begin(), folders.end());  // add user features folders

    ALICEVISION_LOG_DEBUG("List of provided feature folders:");
    for (auto it = folders.begin(); it != folders.end(); ++it)
        ALICEVISION_LOG_DEBUG("\t - " << *it);

    ALICEVISION_LOG_DEBUG("List of feature folders (from sfmData) to load:");
    for (auto it = featuresFolders.begin(); it != featuresFolders.end(); ++it)
        ALICEVISION_LOG_DEBUG("\t - " << *it);

    auto progressDisplay = system::createConsoleProgressDisplay(sfmData.getViews().size(), std::cout, "Loading features\n");

    // read for each view the corresponding features and store them as PointFeatures
    std::atomic_bool invalid(false);

    std::vector<std::unique_ptr<feature::ImageDescriber>> imageDescribers;
    imageDescribers.resize(imageDescriberTypes.size());

    for (std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
        imageDescribers.at(i) = createImageDescriber(imageDescriberTypes.at(i));

#pragma omp parallel
    for (auto iter = sfmData.getViews().begin(); (iter != sfmData.getViews().end()) && (!invalid); ++iter)
    {
#pragma omp single nowait
        {
            for (std::size_t i = 0; i < imageDescriberTypes.size(); ++i)
            {
                std::unique_ptr<feature::Regions> regionsPtr;
                try
                {
                    regionsPtr = loadFeatures(featuresFolders, iter->second.get()->getViewId(), *imageDescribers.at(i));
                }
                catch (const std::exception& e)
                {
                    invalid = true;
#pragma omp critical
                    {
                        ALICEVISION_LOG_ERROR(e.what());
                    }
                    continue;
                }
#pragma omp critical
                {
                    // save loaded Features as PointFeature
                    featuresPerView.addFeatures(iter->second.get()->getViewId(), imageDescriberTypes[i], regionsPtr->GetRegionsPositions());
                    ++progressDisplay;
                }
            }
        }
    }
    return !invalid;
}

}  // namespace sfm
}  // namespace aliceVision
