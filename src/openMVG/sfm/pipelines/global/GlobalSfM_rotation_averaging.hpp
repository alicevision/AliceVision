// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SFM_GLOBAL_ENGINE_PIPELINES_GLOBAL_ROTATION_AVERAGING_HPP
#define OPENMVG_SFM_GLOBAL_ENGINE_PIPELINES_GLOBAL_ROTATION_AVERAGING_HPP

namespace aliceVision{
namespace sfm{

enum ERotationAveragingMethod
{
  ROTATION_AVERAGING_L1 = 1,
  ROTATION_AVERAGING_L2 = 2
};

enum ERelativeRotationInferenceMethod
{
  TRIPLET_ROTATION_INFERENCE_NONE = 0,
  TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR = 1
};

} // namespace sfm
} // namespace aliceVision

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/graph/graph.hpp"
#include "aliceVision/multiview/rotation_averaging_common.hpp"

namespace aliceVision{
namespace sfm{

class GlobalSfM_Rotation_AveragingSolver
{
private:
  mutable Pair_Set used_pairs; // pair that are considered as valid by the rotation averaging solver

public:
  bool Run(
    ERotationAveragingMethod eRotationAveragingMethod,
    ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod,
    const rotation_averaging::RelativeRotations & relativeRot_In,
    Hash_Map<IndexT, Mat3> & map_globalR
  ) const;

  /// Reject edges of the view graph that do not produce triplets with tiny
  ///  angular error once rotation composition have been computed.
  void TripletRotationRejection(
    const double max_angular_error,
    std::vector< graph::Triplet > & vec_triplets,
    rotation_averaging::RelativeRotations & relativeRotations) const;

  /// Return the pairs validated by the GlobalRotation routine (inference can remove some)
  Pair_Set GetUsedPairs() const;
};

} // namespace sfm
} // namespace aliceVision

#endif // OPENMVG_SFM_GLOBAL_ENGINE_PIPELINES_GLOBAL_ROTATION_AVERAGING_HPP
