// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "VocabularyTree.hpp"

#include <algorithm>

namespace aliceVision {
namespace voctree {

float sparseDistance(const SparseHistogram& v1, const SparseHistogram& v2, const std::string &distanceMethod, const std::vector<float>& word_weights)
{

  float distance{0.0f};
  const float epsilon{0.001f};
  
  auto i1 = v1.cbegin();
  auto i1e = v1.cend();
  auto i2 = v2.cbegin();
  auto i2e = v2.cend();
  
  if(distanceMethod == "classic")
  {      
    while(i1 != i1e && i2 != i2e)
    {
      if(i2->first < i1->first)
      {
        distance += i2->second.size();
        ++i2;
      }
      else if(i1->first < i2->first)
      {
        distance += i1->second.size();
        ++i1;
      }
      else
      {
        const auto val = std::minmax(i1->second.size(), i2->second.size());
        distance += static_cast<float>(val.second - val.first);
        ++i1;
        ++i2;
      }
    }

    while(i1 != i1e)
    {
      distance += i1->second.size();
      ++i1;
    }

    while(i2 != i2e)
    {
      distance += i2->second.size();
      ++i2;
    }
  }
  
  else if(distanceMethod == "commonPoints")
  {
    float score{0.f};
    float N1{0.f};
    float N2{0.f};
    
    while(i1 != i1e && i2 != i2e)
    {
      if(i2->first < i1->first)
      {
        N2 += i2->second.size();
        ++i2;
      }
      else if(i1->first < i2->first)
      {
        N1 += i1->second.size();
         ++i1;
      }
      else
      {
        score += std::min(i1->second.size(), i2->second.size());
        N1 += i1->second.size();
        N2 += i2->second.size();
        ++i1;
        ++i2;
      }
    }

    while(i1 != i1e)
    {
      N1 += i1->second.size();
      ++i1;
    }

    while(i2 != i2e)
    {
      N2 += i2->second.size();
      ++i2;
    }
    
    distance = - score;
  }
  
  else if(distanceMethod == "strongCommonPoints")
  {
    float score{0.f};
    float N1{0.f};
    float N2{0.f};
    
    while(i1 != i1e && i2 != i2e)
    {
      if(i2->first < i1->first)
      {
        N2 += i2->second.size();
        ++i2;
      }
      else if(i1->first < i2->first)
      {
        N1 += i1->second.size();
        ++i1;
      }
      else
      {
        if( ( fabs(i1->second.size() - 1.f) < epsilon ) && ( fabs(i2->second.size() - 1.f) < epsilon) )
        {
          score += 1;
          N1 += 1;
          N2 += 1;
        }
        ++i1;
        ++i2;
      }
    }

    while(i1 != i1e)
    {
      N1 += i1->second.size();
      ++i1;
    }

    while(i2 != i2e)
    {
      N2 += i2->second.size();
      ++i2;
    }
    
    distance = - score;
  }
  
  else if(distanceMethod == "weightedStrongCommonPoints")
  {
    float score{0.f};
    float N1{0.f};
    float N2{0.f};
    
    while(i1 != i1e && i2 != i2e)
    {
      if(i2->first < i1->first)
      {
        N2 += i2->second.size()*word_weights[i2->first];
        ++i2;
      }
      else if(i1->first < i2->first)
      {
        N1 += i1->second.size()*word_weights[i1->first];
         ++i1;
      }
        if( ( fabs(i1->second.size() - 1.f) < epsilon ) && ( fabs(i2->second.size() - 1.f) < epsilon) )
        {
          score += word_weights[i1->first];
          N1 += word_weights[i1->first];
          N2 += word_weights[i2->first];
        }
        ++i1;
        ++i2;
    }

    while(i1 != i1e)
    {
      N1 += i1->second.size()*word_weights[i1->first];
      ++i1;
    }

    while(i2 != i2e)
    {
      N2 += i2->second.size()*word_weights[i2->first];
      ++i2;
    }
    
    distance = - score;
  }
  
  else if(distanceMethod == "inversedWeightedCommonPoints")
  {
    float score{0.f};
    float N1{0.f};
    float N2{0.f};
    std::map<int,int> counter;
    
    while(i1 != i1e && i2 != i2e)
    {
      if(i2->first < i1->first)
      {
        N2 += i2->second.size() / word_weights[i2->first];
        ++i2;
      }
      else if(i1->first < i2->first)
      {
        N1 += i1->second.size() / word_weights[i1->first];
         ++i1;
      }
      else
      {
        counter[i1->first] += std::min(i1->second.size(), i2->second.size());
        N1 += i1->second.size() / word_weights[i1->first];
        N2 += i2->second.size() / word_weights[i2->first];
        ++i1;
        ++i2;
      }
    }

    while(i1 != i1e)
    {
      N1 += i1->second.size() / word_weights[i1->first];
      ++i1;
    }

    while(i2 != i2e)
    {
      N2 += i2->second.size() / word_weights[i2->first];
      ++i2;
    }
    
    for(const auto elem : counter)
      score += (1.f/ elem.second) * word_weights[elem.first];
    
    distance = - score;
  }
  else
  {
    throw std::invalid_argument("distance method "+ distanceMethod +" unknown!");
  }
  
  return distance;
}

} //namespace voctree
} //namespace aliceVision