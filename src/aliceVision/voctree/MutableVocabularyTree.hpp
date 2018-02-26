// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "VocabularyTree.hpp"

namespace aliceVision {
namespace voctree {

/**
 * @brief Vocabulary tree that exposes the hierarchical clustering centers. Mainly
 * intended for building a new tree.
 *
 * When loading and using an existing vocabulary tree, use VocabularyTree instead.
 */
template<class Feature, template<typename, typename> class Distance = L2,
class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class MutableVocabularyTree : public VocabularyTree<Feature, Distance, FeatureAllocator>
{
  typedef VocabularyTree<Feature, Distance, FeatureAllocator> BaseClass;

public:
  MutableVocabularyTree()
  {
  }

  void setSize(uint32_t levels, uint32_t splits)
  {
    this->levels_ = levels;
    this->k_ = splits;
    this->setNodeCounts();
  }

  uint32_t nodes() const
  {
    return this->word_start_ + this->num_words_;
  }

  std::vector<Feature, FeatureAllocator>& centers()
  {
    return this->centers_;
  }

  const std::vector<Feature, FeatureAllocator>& centers() const
  {
    return this->centers_;
  }

  std::vector<uint8_t>& validCenters()
  {
    return this->valid_centers_;
  }

  const std::vector<uint8_t>& validCenters() const
  {
    return this->valid_centers_;
  }
};

}
}
