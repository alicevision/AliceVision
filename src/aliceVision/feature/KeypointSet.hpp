// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ALICEVISION_FEATURES_KEYPOINTSET_HPP
#define ALICEVISION_FEATURES_KEYPOINTSET_HPP

#include "aliceVision/feature/PointFeature.hpp"
#include "aliceVision/feature/Descriptor.hpp"
#include <string>

namespace aliceVision {
namespace feature {

/// Association storage of associated feature and descriptor for a given image.
/// Load, save, R/W accessor operation.
///
/// typedef vector<PointFeature> featsT;
/// typedef vector<Descriptor<uchar,128> > descsT;
/// KeypointSet< featsT, descsT > kpSet;
template<typename FeaturesT, typename DescriptorsT>
class KeypointSet {
public:
  // Alias to stored Feature and Descriptor type
  typedef typename FeaturesT::value_type FeatureT;
  typedef typename DescriptorsT::value_type DescriptorT;

  /// Read from files the feats and their corresponding descriptors.
  void loadFromFile(
    vfs::filesystem& fs,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs)
  {
    loadFeatsFromFile(fs, sfileNameFeats, _feats);
    loadDescsFromFile(fs, sfileNameDescs, _descs);
  }

  /// Export in two separate files the feats and their corresponding descriptors.
  void saveToFile(
    vfs::filesystem& fs,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    saveFeatsToFile(fs, sfileNameFeats, _feats);
    saveDescsToFile(fs, sfileNameDescs, _descs);
  }

  /// Read from files the feats and their corresponding descriptors
  ///  descriptor in binary to save place
  void loadFromBinFile(
    vfs::filesystem& fs,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs)
  {
    loadFeatsFromFile(fs, sfileNameFeats, _feats);
    loadDescsFromBinFile(fs, sfileNameDescs, _descs);
  }

  /// Export in two separate files the feats and their corresponding descriptors
  ///  descriptor in binary to save place
  void saveToBinFile(
    vfs::filesystem& fs,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    saveFeatsToFile(fs, sfileNameFeats, _feats);
    saveDescsToBinFile(fs, sfileNameDescs, _descs);
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

} // namespace feature
} // namespace aliceVision

#endif // ALICEVISION_FEATURES_KEYPOINTSET_HPP
