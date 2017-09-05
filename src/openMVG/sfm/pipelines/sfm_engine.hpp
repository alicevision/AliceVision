// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "openMVG/sfm/sfm_data.hpp"
#include <string>

namespace openMVG {
namespace sfm {

/// Basic Reconstruction Engine.
/// Process Function handle the reconstruction.
class ReconstructionEngine
{
public:

  ReconstructionEngine(
    const SfM_Data & sfm_data,
    const std::string & soutDirectory)
    :_sOutDirectory(soutDirectory),
    _sfm_data(sfm_data),
    _bFixedIntrinsics(false)
  {
  }

  virtual ~ReconstructionEngine() {}

  virtual bool Process() = 0;

  bool Get_bFixedIntrinsics() const {return _bFixedIntrinsics;}
  void Set_bFixedIntrinsics(bool bVal) {_bFixedIntrinsics = bVal;}

  SfM_Data & Get_SfM_Data() { return _sfm_data; }
  const SfM_Data & Get_SfM_Data() const { return _sfm_data; }

  bool Colorize() { return ColorizeTracks(_sfm_data); }

protected:
  std::string _sOutDirectory; // Output path where outputs will be stored

  //-----
  //-- Reconstruction data
  //-----
  SfM_Data _sfm_data; // internal SfM_Data

  //-----
  //-- Reconstruction parameters
  //-----
  bool _bFixedIntrinsics;
};

} // namespace sfm
} // namespace openMVG
