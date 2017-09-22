// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "pairBuilder.hpp"

#include <aliceVision/stl/split.hpp>
#include <aliceVision/system/Logger.hpp>

#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
namespace aliceVision {

/// Generate all the (I,J) pairs of the upper diagonal of the NxN matrix
PairSet exhaustivePairs(const sfm::Views& views, int rangeStart, int rangeSize)
{
  PairSet pairs;
  sfm::Views::const_iterator itA = views.begin();
  sfm::Views::const_iterator itAEnd = views.end();

  // If we have a rangeStart, only compute the matching for (rangeStart, X).
  if(rangeStart != -1 && rangeSize != 0)
  {
    if(rangeStart >= views.size())
      return pairs;
    std::advance(itA, rangeStart);
    itAEnd = views.begin();
    std::advance(itAEnd, std::min(std::size_t(rangeStart+rangeSize), views.size()));
  }

  for(; itA != itAEnd; ++itA)
  {
    sfm::Views::const_iterator itB = itA;
    std::advance(itB, 1);
    for(; itB != views.end(); ++itB)
      pairs.insert(std::make_pair(itA->first, itB->first));
  }
  return pairs;
}

/// Generate the pairs that have a distance inferior to the overlapSize
/// Usable to match video sequence
PairSet contiguousWithOverlap(const sfm::Views& views, const size_t overlapSize, int rangeStart, int rangeSize)
{
  PairSet pairs;
  sfm::Views::const_iterator itA = views.begin();
  sfm::Views::const_iterator itAEnd = views.end();

  // If we have a rangeStart, only compute the matching for (rangeStart, X).
  if(rangeStart != -1 && rangeSize != 0)
  {
    if(rangeStart >= views.size())
      return pairs;
    std::advance(itA, rangeStart);
    itAEnd = views.begin();
    std::advance(itAEnd, std::min(std::size_t(rangeStart+rangeSize), views.size()));
  }

  for(; itA != itAEnd; ++itA)
  {
    sfm::Views::const_iterator itB = itA;
    std::advance(itB, 1);
    sfm::Views::const_iterator itBEnd = itA;
    std::size_t advanceSize = overlapSize;
    if(std::distance(itBEnd, views.end()) < 1 + overlapSize)
    {
      advanceSize = std::distance(itBEnd, views.end()) - 1;
    }
    std::advance(itBEnd, 1 + advanceSize);

    for(; itB != views.end() && itB != itBEnd; ++itB)
      pairs.insert(std::make_pair(itA->first, itB->first));
  }
  return pairs;
}

bool loadPairs(const std::string &sFileName,
               PairSet & pairs,
               int rangeStart,
               int rangeSize)
{
  std::ifstream in(sFileName.c_str());
  if(!in.is_open())
  {
    ALICEVISION_LOG_WARNING("loadPairs: Impossible to read the specified file: \"" << sFileName << "\".");
    return false;
  }
  std::size_t nbLine = 0;
  std::string sValue;
  std::vector<std::string> vec_str;
  for(; std::getline( in, sValue ); ++nbLine)
  {
    if(rangeStart != -1 && rangeSize != 0)
    {
      if(nbLine < rangeStart)
        continue;
      if(nbLine >= rangeStart + rangeSize)
        break;
    }
    vec_str.clear();
    stl::split(sValue, " ", vec_str);
    const size_t str_size = vec_str.size();
    if (str_size < 2)
    {
      ALICEVISION_LOG_WARNING("loadPairs: Invalid input file: \"" << sFileName << "\".");
      return false;
    }
    std::stringstream oss;
    oss.clear(); oss.str(vec_str[0]);
    size_t I, J;
    oss >> I;
    for(size_t i=1; i<str_size ; ++i)
    {
      oss.clear(); oss.str(vec_str[i]);
      oss >> J;
      if( I == J )
      {
        ALICEVISION_LOG_WARNING("loadPairs: Invalid input file. Image " << I << " see itself. File: \"" << sFileName << "\".");
        return false;
      }
      Pair pairToInsert = (I < J) ? std::make_pair(I, J) : std::make_pair(J, I);
      if(pairs.find(pairToInsert) != pairs.end())
      {
        // There is no reason to have the same image pair twice in the list of image pairs to match.
        ALICEVISION_LOG_WARNING("loadPairs: Image pair " << I << ", " << J << " already added. File: \"" << sFileName << "\".");
      }
      pairs.insert(pairToInsert);
    }
  }
  in.close();
  return true;
}

bool savePairs(const std::string &sFileName, const PairSet & pairs)
{
  std::ofstream outStream(sFileName.c_str());
  if(!outStream.is_open())  {
    ALICEVISION_LOG_WARNING("savePairs: Impossible to open the output specified file: \"" << sFileName << "\".");
    return false;
  }
  for (PairSet::const_iterator iterP = pairs.begin();
    iterP != pairs.end(); ++iterP)
  {
    outStream << iterP->first << ' ' << iterP->second << '\n';
  }
  bool bOk = !outStream.bad();
  outStream.close();
  return bOk;
}

}; // namespace aliceVision
