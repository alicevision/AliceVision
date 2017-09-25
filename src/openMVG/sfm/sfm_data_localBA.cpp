
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/sfm/sfm_data_localBA.hpp"

namespace openMVG {
namespace sfm {

LocalBA_Data::LocalBA_Data(const SfM_Data& sfm_data)
{
  for (const auto& it : sfm_data.intrinsics)
  {
    intrinsicsHistory[it.first];
    intrinsicsHistory.at(it.first).push_back(std::make_pair(0, sfm_data.GetIntrinsicPtr(it.first)->getParams()));
    intrinsicsLimitIds[it.first];
    intrinsicsLimitIds.at(it.first) = std::vector<IndexT> (3, 0); 
  }
}

void LocalBA_Data::addIntrinsicsToHistory(const SfM_Data& sfm_data)
{
  std::map<IndexT, std::size_t> map_intrId_numPoses = sfm_data.GetIntrinsicsUsage();
  
  for (auto& it : sfm_data.intrinsics)
  {
    intrinsicsHistory.at(it.first).push_back(
          std::make_pair(map_intrId_numPoses[it.first],
          sfm_data.GetIntrinsicPtr(it.first)->getParams())
        );
  }
}

// focal: parameterId = 0
// Cx: parameterId = 1
// Cy: parameterId = 2
void LocalBA_Data::computeParameterLimits(const IntrinsicParameter& parameter, const std::size_t kWindowSize, const double kStdDevPercentage)
{
  std::cout << "Updating parameter #" << parameter << std::endl;
  for (auto& elt : intrinsicsHistory)
  {
    IndexT idIntr = elt.first;
    
    // Do not compute limits if there are already reached for each parameter
    if (intrinsicsLimitIds.at(idIntr).at(parameter) != 0)
      continue;
    
    
    // Get the full history of intrinsic parameters
    std::vector<std::size_t> allNumPosesVec;
    std::vector<double> allValuesVec; 
    
    for (const auto& pair_uses_params : intrinsicsHistory.at(idIntr))
    {
      allNumPosesVec.push_back(pair_uses_params.first);
      allValuesVec.push_back(pair_uses_params.second.at(parameter));
    }
    
    std::cout << "- Clean duplicated & removed cameras..." << std::endl;
    // Clean 'intrinsicsHistorical':
    //  [4 5 5 7 8 6 9]
    // - detect duplicates -> [4 (5) 5 7 8 6 9]
    // - detecting removed cameras -> [4 5 (7 8) 6 9]
    std::vector<std::size_t> filteredNumPosesVec(allNumPosesVec);
    std::vector<double> filteredValuesVec(allValuesVec);
    
    std::size_t numPosesEndWindow = allNumPosesVec.back();
    
    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
    {
      if (filteredNumPosesVec.size() < 2)
        break;
      
      if (filteredNumPosesVec.at(id) >= filteredNumPosesVec.at(id+1))
      {
        filteredNumPosesVec.erase(filteredNumPosesVec.begin()+id);
        filteredValuesVec.erase(filteredValuesVec.begin()+id);
      }
    }
    
    /* Display info */
    std::cout << "-- K #" << idIntr << std::endl;
    std::cout << "allNumPosesVec = " << allNumPosesVec << std::endl;
    std::cout << "allValuesVec = " << allValuesVec << std::endl;
    std::cout << "filteredValuesVec = " << filteredValuesVec << std::endl;
    
    // Detect limit according to 'kWindowSize':
    if (numPosesEndWindow < kWindowSize)
      continue;
    
    IndexT idStartWindow = 0;
    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
    {
      if (numPosesEndWindow - filteredNumPosesVec.at(id) >= kWindowSize)
      {
        idStartWindow = id;
        break;
      }
    }
    
    std::size_t numPosesStartWindow = filteredNumPosesVec.at(idStartWindow);
    
    // Normalize parameters historical:
    // The normalization need to be done on the all historical
    std::cout << "- Normalize..." << std::endl;
    std::vector<double> normalizedValuesVec = normalize(filteredValuesVec);
    // Compute the standard deviation for each parameter, between [idLimit; end()]
    std::cout << "- Subpart vector..." << std::endl;
    std::vector<double> subNumPosesVec (filteredNumPosesVec.begin()+idStartWindow, filteredNumPosesVec.end());
    std::vector<double> subNormValuesVec (normalizedValuesVec.begin()+idStartWindow, normalizedValuesVec.end());
    std::cout << "- Compute stdev..." << std::endl;
    double stdevSubValues = standardDeviation(subNormValuesVec);
    
    /* Display info */
    std::cout << "filtredNumPosesVec = " << filteredNumPosesVec << std::endl;
    std::cout << "idStartWindow = " << idStartWindow << std::endl;
    std::cout << "numPosesStartWindow = " << numPosesStartWindow << std::endl;
    std::cout << "numPosesEndWindow = " << numPosesEndWindow << std::endl;
    std::cout << "subNumPosesVec = " << subNumPosesVec << std::endl;
    std::cout << "normalizedValuesVec = " << normalizedValuesVec << std::endl;
    std::cout << "subNormValuesVec = " << subNormValuesVec << std::endl;
    std::cout << "stdevSubValues = " << stdevSubValues << std::endl;
    
    if (stdevSubValues*100.0 <= kStdDevPercentage && intrinsicsLimitIds.at(idIntr).at(parameter) == 0)
    {
      intrinsicsLimitIds.at(idIntr).at(parameter) = numPosesEndWindow;    
      getchar();
    }
  }
}

void LocalBA_Data::computeAllParametersLimits(const std::size_t kWindowSize, const double kStdDevPercentage)
{
  computeParameterLimits(IntrinsicParameter::Focal, kWindowSize, kStdDevPercentage); 
  computeParameterLimits(IntrinsicParameter::Cx, kWindowSize, kStdDevPercentage); 
  computeParameterLimits(IntrinsicParameter::Cy, kWindowSize, kStdDevPercentage); 
}

template<typename T> 
std::vector<T> LocalBA_Data::normalize(const std::vector<T>& data) 
{ 
  std::vector<T> normalizedData;
  normalizedData.reserve(data.size());
  T minVal = *std::min_element(data.begin(), data.end());
  T maxVal = *std::max_element(data.begin(), data.end());
  for (auto const& val : data)
    normalizedData.push_back((val - minVal)/(maxVal - minVal));
  return normalizedData;
}  

template<typename T> 
double LocalBA_Data::standardDeviation(const std::vector<T>& data) 
{ 
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();
  std::vector<double> diff(data.size());
  std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / data.size());
}  

} // namespace sfm
} // namespace openMVG
