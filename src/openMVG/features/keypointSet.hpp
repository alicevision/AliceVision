// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_FEATURES_KEYPOINTSET_HPP
#define OPENMVG_FEATURES_KEYPOINTSET_HPP

#include "aliceVision/features/feature.hpp"
#include "aliceVision/features/descriptor.hpp"
#include <string>

namespace aliceVision {
namespace features {

/// Association storage of associated feature and descriptor for a given image.
/// Load, save, R/W accessor operation.
///
/// typedef vector<SIOPointFeature> featsT;
/// typedef vector<Descriptor<uchar,128> > descsT;
/// KeypointSet< featsT, descsT > kpSet;
template<typename FeaturesT, typename DescriptorsT>
class KeypointSet {
public:
  // Alias to stored Feature and Descriptor type
  typedef typename FeaturesT::value_type FeatureT;
  typedef typename DescriptorsT::value_type DescriptorT;

  /// Read from files the feats and their corresponding descriptors.
  bool loadFromFile(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs)
  {
    return loadFeatsFromFile(sfileNameFeats, _feats)
          & loadDescsFromFile(sfileNameDescs, _descs);
  }

  /// Export in two separate files the feats and their corresponding descriptors.
  bool saveToFile(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    return saveFeatsToFile(sfileNameFeats, _feats)
          & saveDescsToFile(sfileNameDescs, _descs);
  }

  /// Read from files the feats and their corresponding descriptors
  ///  descriptor in binary to save place
  bool loadFromBinFile(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs)
  {
    return loadFeatsFromFile(sfileNameFeats, _feats)
          & loadDescsFromBinFile(sfileNameDescs, _descs);
  }

  /// Export in two separate files the feats and their corresponding descriptors
  ///  descriptor in binary to save place
  bool saveToBinFile(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    return saveFeatsToFile(sfileNameFeats, _feats)
          & saveDescsToBinFile(sfileNameDescs, _descs);
  }

  /// Mutable and non-mutable FeatureT getters.
  inline FeaturesT & features() { return _feats; }
  inline const FeaturesT & features() const { return _feats; }

  /// Mutable and non-mutable DescriptorT getters.
  inline DescriptorsT & descriptors() { return _descs; }
  inline const DescriptorsT & descriptors() const { return _descs; }

private:
  FeaturesT _feats;
  DescriptorsT _descs;
};

} // namespace features
} // namespace aliceVision

#endif // OPENMVG_FEATURES_KEYPOINTSET_HPP
