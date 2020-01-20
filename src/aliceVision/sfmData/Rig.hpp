// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#include <vector>
#include <cassert>
#include <algorithm>
#include <stdexcept>

namespace aliceVision {
namespace sfmData {

/**
 * @brief Rig sub-pose status enum
 */
enum class ERigSubPoseStatus: std::uint8_t
{
  UNINITIALIZED = 0
  , ESTIMATED = 1
  , CONSTANT = 2
};

/**
 * @brief convert an enum ERigSubPoseStatus to its corresponding string
 * @param ERigSubPoseStatus
 * @return String
 */
inline std::string ERigSubPoseStatus_enumToString(ERigSubPoseStatus rigSubPoseStatus)
{
  switch(rigSubPoseStatus)
  {
    case ERigSubPoseStatus::UNINITIALIZED: return "uninitialized";
    case ERigSubPoseStatus::ESTIMATED:     return "estimated";
    case ERigSubPoseStatus::CONSTANT:      return "constant";
  }
  throw std::out_of_range("Invalid rigSubPoseStatus enum");
}

/**
 * @brief convert a string rigSubPoseStatus to its corresponding enum ERigSubPoseStatus
 * @param String
 * @return ERigSubPoseStatus
 */
inline ERigSubPoseStatus ERigSubPoseStatus_stringToEnum(const std::string& rigSubPoseStatus)
{
  std::string status = rigSubPoseStatus;
  std::transform(status.begin(), status.end(), status.begin(), ::tolower); //tolower

  if(status == "uninitialized") return ERigSubPoseStatus::UNINITIALIZED;
  if(status == "estimated")     return ERigSubPoseStatus::ESTIMATED;
  if(status == "constant")      return ERigSubPoseStatus::CONSTANT;

  throw std::out_of_range("Invalid rigSubPoseStatus : " + rigSubPoseStatus);
}

struct RigSubPose
{
  /// status of the sub-pose
  ERigSubPoseStatus status = ERigSubPoseStatus::UNINITIALIZED;
  /// relative pose of the sub-pose
  geometry::Pose3 pose;

  /**
   * @brief RigSubPose constructor
   * @param pose The relative pose of the sub-pose
   * @param status The status of the sub-pose
   */
  RigSubPose(const geometry::Pose3& pose = geometry::Pose3(),
             ERigSubPoseStatus status = ERigSubPoseStatus::UNINITIALIZED)
    : pose(pose)
    , status(status)
  {}

  /**
   * @brief operator ==
   * @param other the other RigSubPose
   * @return true if the current RigSubPose and the other are identical.
   */
  bool operator==(const RigSubPose& other) const
  {
    return (status == other.status &&
            pose == other.pose);
  }
};

class Rig
{
public:

  /**
   * @brief Rig constructor
   * @param nbSubPoses The number of sub-poses of the rig
   */
  explicit Rig(unsigned int nbSubPoses = 0)
  {
    _subPoses.resize(nbSubPoses);
  }


  /**
   * @brief operator ==
   * @param other the other Rig
   * @return true if the current rig and the other are identical.
   */
  bool operator==(const Rig& other) const
  {
    return _subPoses == other._subPoses;
  }

  /**
   * @brief Check if the rig has at least one sub-pose initialized
   * @return true if at least one subpose initialized
   */
  bool isInitialized() const
  {
    for(const RigSubPose& subPose : _subPoses)
    {
      if(subPose.status != ERigSubPoseStatus::UNINITIALIZED)
        return true;
    }
    return false;
  }

  bool isFullyCalibrated() const
  {
    for(const RigSubPose& subPose : _subPoses)
    {
      if(subPose.status == ERigSubPoseStatus::UNINITIALIZED)
        return false;
    }
    return true;
  }

  /**
   * @brief Get the number of sub-poses in the rig
   * @return number of sub-poses in the rig
   */
  std::size_t getNbSubPoses() const
  {
    return _subPoses.size();
  }

  /**
   * @brief Get the sub-poses const vector
   * @return rig sub-poses
   */
  const std::vector<RigSubPose>& getSubPoses() const
  {
    return _subPoses;
  }

  /**
  * @brief Get the sub-poses const vector
  * @return rig sub-poses
  */
  std::vector<RigSubPose>& getSubPoses()
  {
      return _subPoses;
  }
  /**
   * @brief Get the sub-pose for the given sub-pose index
   * @param index The sub-pose index
   * @return corresponding rig sub-pose
   */
  const RigSubPose& getSubPose(IndexT index) const
  {
    return _subPoses.at(index);
  }

  /**
   * @brief Get the sub-pose for the given sub-pose index
   * @param index The sub-pose index
   * @return corresponding rig sub-pose
   */
  RigSubPose& getSubPose(IndexT index)
  {
    return _subPoses.at(index);
  }

  /**
   * @brief Set the given sub-pose for the given sub-pose index
   * @param index The sub-pose index
   * @param rigSubPose The rig sub-pose
   */
  void setSubPose(IndexT index, const RigSubPose& rigSubPose)
  {
    assert(_subPoses.size() > index);
    _subPoses[index] = rigSubPose;
  }

  /**
   * @brief Reset all sub-poses parameters
   */
  void reset()
  {
    for(RigSubPose& subPose : _subPoses)
    {
      subPose.status = ERigSubPoseStatus::UNINITIALIZED;
      subPose.pose = geometry::Pose3();
    }
  }

private:

  /// rig sub-poses
  std::vector<RigSubPose> _subPoses;
};

} // namespace sfmData
} // namespace aliceVision
