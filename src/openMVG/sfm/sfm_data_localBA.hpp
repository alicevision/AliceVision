// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_LOCALBA_HPP
#define OPENMVG_SFM_DATA_LOCALBA_HPP

#include "openMVG/types.hpp"
#include "openMVG/sfm/sfm_data.hpp"

namespace openMVG {
namespace sfm {

/// Intrinsic parameters 
/// <NbOfPosesWithACommonIntrinsic, IntrinsicParameters>
using IntrinsicParams = std::pair<std::size_t, std::vector<double>>;

/// Save the progression for all the intrinsics parameters
/// <IntrinsicId, IntrinsicsParametersHistory>
using IntrinicsHistory = std::map<IndexT, std::vector<IntrinsicParams>>;

class LocalBA_Data
{
  IntrinicsHistory intrinsicsHistory; // Backup of the intrinsics parameters
  
  std::map<IndexT, std::vector<IndexT>> intrinsicsLimitIds; // <IntrinsicIndex, <F_limitId, CX_limitId, CY_limitId>>
  
public:
  
  LocalBA_Data(const SfM_Data& sfm_data);
  
  enum IntrinsicParameter{Focal, Cx, Cy};
  
  void addIntrinsicsToHistory(const SfM_Data& sfm_data);
  
  void computeParameterLimits(const IntrinsicParameter &parameter, const std::size_t kWindowSize, const double kStdDevPercentage);

  void computeAllParametersLimits(const std::size_t kWindowSize, const double kStdDevPercentage);
  
  std::vector<IndexT> getIntrinsicLimitIds(const IndexT intrinsicId) const {return intrinsicsLimitIds.at(intrinsicId);}
  
  bool isLimitReached(const IndexT intrinsicId, IntrinsicParameter parameter) const { return intrinsicsLimitIds.at(intrinsicId).at(parameter) != 0;}
  
private:
  /// Normalize data as: 
  /// normalizedData[i] = (data[i] - min(data)) / (max(data) - min(data)) 
  template<typename T> 
  std::vector<T> normalize(const std::vector<T>& data);
  
  template<typename T> 
  double standardDeviation(const std::vector<T>& data);
};



} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_LOCAL_HPP
