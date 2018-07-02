// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/pipeline/localization/SfMLocalizationSingle3DTrackObservationDatabase.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"

namespace aliceVision {
namespace sfm {

  SfMLocalizationSingle3DTrackObservationDatabase::
  SfMLocalizationSingle3DTrackObservationDatabase()
  :SfMLocalizer(), sfm_data_(nullptr), matching_interface_(nullptr)
  {}

  bool
  SfMLocalizationSingle3DTrackObservationDatabase::Init
  (
    const SfMData & sfm_data,
    const feature::RegionsPerView & regionsPerView
  )
  {
    if (regionsPerView.isEmpty())
    {
      return false;
    }

    if (sfm_data.getPoses().empty() || sfm_data.getLandmarks().empty())
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

  bool
  SfMLocalizationSingle3DTrackObservationDatabase::Localize
  (
    const Pair & image_size,
    const camera::IntrinsicBase * optional_intrinsics,
    const feature::Regions & query_regions,
    geometry::Pose3 & pose,
    ImageLocalizerMatchData * resection_data_ptr
  ) const
  {
    if (sfm_data_ == nullptr || matching_interface_ == nullptr)
    {
      return false;
    }

    matching::IndMatches vec_putative_matches;
    if (!matching_interface_->Match(0.8, query_regions, vec_putative_matches))
    {
      return false;
    }

    ALICEVISION_LOG_DEBUG("#3D2d putative correspondences: " << vec_putative_matches.size());
    // Init the 3D-2d correspondences array
    ImageLocalizerMatchData resection_data;
    if (resection_data_ptr)
    {
      resection_data.error_max = resection_data_ptr->error_max;
    }
    resection_data.pt3D.resize(3, vec_putative_matches.size());
    resection_data.pt2D.resize(2, vec_putative_matches.size());
    for (size_t i = 0; i < vec_putative_matches.size(); ++i)
    {
      resection_data.pt3D.col(i) = sfm_data_->getLandmarks().at(index_to_landmark_id_[vec_putative_matches[i]._i]).X;
      resection_data.pt2D.col(i) = query_regions.GetRegionPosition(vec_putative_matches[i]._j);
    }

    const bool bResection =  SfMLocalizer::Localize(
      image_size, optional_intrinsics, resection_data, pose);

    if (resection_data_ptr != nullptr)
      (*resection_data_ptr) = std::move(resection_data);

    return bResection;
  }

} // namespace sfm
} // namespace aliceVision
