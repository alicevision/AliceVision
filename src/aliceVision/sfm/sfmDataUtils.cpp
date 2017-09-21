// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "sfmDataUtils.hpp"
#include <aliceVision/sfm/SfMData.hpp>

namespace aliceVision {
namespace sfm {

void GroupSharedIntrinsics(SfMData & sfm_data)
{
  Views & views = sfm_data.views;
  Intrinsics & intrinsics = sfm_data.intrinsics;

  // Build hash & build a set of the hash in order to maintain unique Ids
  std::set<size_t> hash_index;
  std::vector<size_t> hash_value;

  for (Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
    iterIntrinsic != intrinsics.end();
    ++iterIntrinsic)
  {
    const camera::IntrinsicBase * intrinsicData = iterIntrinsic->second.get();
    const size_t hashVal = intrinsicData->hashValue();
    hash_index.insert(hashVal);
    hash_value.push_back(hashVal);
  }

  // From hash_value(s) compute the new index (old to new indexing)
  Hash_Map<IndexT, IndexT> old_new_reindex;
  size_t i = 0;
  for (Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
    iterIntrinsic != intrinsics.end();
    ++iterIntrinsic, ++i)
  {
    old_new_reindex[iterIntrinsic->first] = std::distance(hash_index.begin(), hash_index.find(hash_value[i]));
  }
  //--> Save only the required Intrinsics (do not need to keep all the copy)
  Intrinsics intrinsic_updated;
  for (Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
    iterIntrinsic != intrinsics.end();
    ++iterIntrinsic)
  {
    intrinsic_updated[old_new_reindex[iterIntrinsic->first]] = intrinsics[iterIntrinsic->first];
  }
  // Update intrinsics (keep only the necessary ones) -> swapping
  intrinsics.swap(intrinsic_updated);

  // Update views intrinsic IDs (since some intrinsic position have changed in the map)
  for (Views::iterator iterView = views.begin();
    iterView != views.end();
    ++iterView)
  {
    View * v = iterView->second.get();
    // Update the Id only if a corresponding index exists
    if (old_new_reindex.count(v->getIntrinsicId()))
      v->setIntrinsicId(old_new_reindex[v->getIntrinsicId()]);
  }
}

} // namespace sfm
} // namespace aliceVision
