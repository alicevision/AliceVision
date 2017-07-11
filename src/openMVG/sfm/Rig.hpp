#pragma once

#include <openMVG/types.hpp>
#include <openMVG/geometry/pose3.hpp>

#include <cereal/cereal.hpp>
#include <cereal/types/map.hpp>

#include <map>

namespace openMVG {
namespace sfm {

enum class ERigSubPoseStatus: std::uint8_t
{
  UNINITIALIZED = 0
  , ESTIMATED = 1
  , CONSTANT = 2
};

struct RigSubPose
{
  ERigSubPoseStatus status = ERigSubPoseStatus::UNINITIALIZED;
  geometry::Pose3 pose;

  /**
   * @brief cereal serialize method
   * @param ar The archive
   */
  template <class Archive>
  void serialize(Archive & ar)
  {
    ar(cereal::make_nvp("status", status),
       cereal::make_nvp("pose", pose));
  }
};

class Rig
{
public:

  Rig(unsigned int nbCameras = 0)
  {
    for(unsigned int i = 0; i < nbCameras; ++i)
    {
      _subPoses[i] = RigSubPose();
    }
  }

  /**
   * @brief Get the subPose for the given subPose index
   * @param index The subPose index
   * @return corresponding RigSubPose
   */
  const RigSubPose& getSubPose(IndexT index) const
  {
    return _subPoses.at(index);
  }

  /**
   * @brief Get the subPose for the given subPose index
   * @param index The subPose index
   * @return corresponding RigSubPose
   */
  RigSubPose& getSubPose(IndexT index)
  {
    return _subPoses.at(index);
  }

  /**
   * @brief cereal serialize method
   * @param ar The archive
   */
  template <class Archive>
  void serialize(Archive & ar)
  {
    ar(cereal::make_nvp("subposes", _subPoses));
  }

private:
  std::map<IndexT, RigSubPose> _subPoses;
};

} // namespace sfm
} // namespace openMVG
