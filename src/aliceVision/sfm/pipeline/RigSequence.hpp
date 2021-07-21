// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/TracksBuilder.hpp>

namespace aliceVision {
namespace sfm {


struct RigParams
{
    bool useRigConstraint = true;
    int minNbCamerasForCalibration = 20;
};

class RigSequence
{
public:

  RigSequence(sfmData::SfMData& sfmData, IndexT rigId, RigParams params)
    : _sfmData(sfmData)
    , _rigId(rigId)
    , _rig(sfmData.getRigs().at(rigId))
    , _params(params)
  {}

  /**
   * @brief RigSequence initialization
   * build internal structures
   */
  void init(const track::TracksPerView& tracksPerView);

  /**
   * @brief Calibrate new possible rigs or update independent poses to rig poses
   * @param[in,out] updatedViews add the updated view ids to the list
   * @return set of updated views id
   */
  void updateSfM(std::set<IndexT>& updatedViews);

private:

  void computeScores();
  void setupRelativePoses();
  void rigResection(std::set<IndexT>& updatedViews);

  struct ViewInfo
  {
    IndexT viewId;
    bool isPoseIndependant;
    double score;
  };

  struct SubPoseInfo
  {
    std::size_t nbPose = 0;
    bool isInitialized = false;
    IndexT maxFrameId = UndefinedIndexT;
    double maxFrameScore = 0;
    double totalScore = 0;
  };

  // <subPoseId, viewInfo>
  using RigFrame = std::map<IndexT, ViewInfo>;
  // <frame, RigFrame>
  using RigInfoPerFrame = std::map<IndexT, RigFrame>;

  IndexT _rigId;
  sfmData::Rig& _rig;
  sfmData::SfMData& _sfmData;
  RigInfoPerFrame _rigInfoPerFrame;
  std::map<IndexT, SubPoseInfo> _rigInfoPerSubPose;
  RigParams _params;
};

IndexT getRigPoseId(IndexT rigId, IndexT frameId);

} // namespace sfm
} // namespace aliceVision
