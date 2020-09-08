// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfMLocalizationSingle3DTrackObservationDatabase.hpp"
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>

namespace aliceVision {
namespace sfm {

  SfMLocalizationSingle3DTrackObservationDatabase::SfMLocalizationSingle3DTrackObservationDatabase()
    : SfMLocalizer()
    , _sfmData(nullptr)
    , _matchingInterface(nullptr)
  {}

  bool SfMLocalizationSingle3DTrackObservationDatabase::Init(const sfmData::SfMData& sfmData,
                                                             const feature::RegionsPerView& regionsPerView)
  {
    if (regionsPerView.isEmpty())
    {
      return false;
    }

    if (sfmData.getPoses().empty() || sfmData.getLandmarks().empty())
    {
      ALICEVISION_LOG_WARNING("The input SfMData file have not 3D content to match with.");
      return false;
    }

    /*
    TODO: DELI

    // Setup the database
    // A collection of regions
    // - each view observation leads to a new regions
    // - link each observation region to a track id to ease 2D-3D correspondences search

    const feature::Regions& regionsType = regionsPerView.getFirstViewRegions();
    landmark_observations_descriptors_.reset(regionsType.EmptyClone());
    for (const auto & landmark : sfm_data.getLandmarks())
    {
      for (const auto & observation : landmark.second.obs)
      {
        if (observation.second.id_feat != UndefinedIndexT)
        {
          // copy the feature/descriptor to landmark_observations_descriptors
          const feature::Regions& viewRegions = regionsPerView.getRegions(observation.first, landmark.second.descType);
          viewRegions.CopyRegion(observation.second.id_feat, landmark_observations_descriptors_.get());
          // link this descriptor to the track Id
          index_to_landmark_id_.push_back(landmark.first);
        }
      }
    }
    ALICEVISION_LOG_DEBUG("Init retrieval database ... ");
    matching_interface_.reset(new
      matching::Matcher_Regions_Database(matching::ANN_L2, *landmark_observations_descriptors_));
    ALICEVISION_LOG_DEBUG("Retrieval database initialized\n"
      "#landmark: " << sfm_data.getLandmarks().size() << "\n"
      "#descriptor initialized: " << landmark_observations_descriptors_->RegionCount());

    sfm_data_ = &sfm_data;
    */

    return true;
  }

  bool SfMLocalizationSingle3DTrackObservationDatabase::Localize(const Pair& imageSize,
                               const camera::IntrinsicBase* optionalIntrinsics,
                               const feature::Regions& queryRegions,
                               std::mt19937 &randomNumberGenerator, 
                               geometry::Pose3& pose,
                               ImageLocalizerMatchData* resectionDataPtr) const
  {
    if(_sfmData == nullptr || _matchingInterface == nullptr)
      return false;

    matching::IndMatches putativeMatches;
    if(!_matchingInterface->Match(0.8, queryRegions, putativeMatches))
      return false;

    ALICEVISION_LOG_DEBUG("#3D2d putative correspondences: " << putativeMatches.size());

    // Init the 3D-2d correspondences array
    ImageLocalizerMatchData resectionData;

    if(resectionDataPtr)
      resectionData.error_max = resectionDataPtr->error_max;

    resectionData.pt3D.resize(3, putativeMatches.size());
    resectionData.pt2D.resize(2, putativeMatches.size());

    for(std::size_t i = 0; i < putativeMatches.size(); ++i)
    {
      resectionData.pt3D.col(i) = _sfmData->getLandmarks().at(_indexToLandmarkId[putativeMatches[i]._i]).X;
      resectionData.pt2D.col(i) = queryRegions.GetRegionPosition(putativeMatches[i]._j);
    }

    const bool resection =  SfMLocalizer::Localize(imageSize, optionalIntrinsics, randomNumberGenerator, resectionData, pose);

    if(resectionDataPtr != nullptr)
      (*resectionDataPtr) = std::move(resectionData);

    return resection;
  }

} // namespace sfm
} // namespace aliceVision
