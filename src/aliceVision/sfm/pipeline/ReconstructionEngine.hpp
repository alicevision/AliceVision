// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/sfm/SfMData.hpp"
#include <string>

namespace aliceVision {
namespace sfm {

/// Basic Reconstruction Engine.
/// Process Function handle the reconstruction.
class ReconstructionEngine
{
public:

  ReconstructionEngine(
    const SfMData & sfm_data,
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

  SfMData & Get_SfMData() { return _sfm_data; }
  const SfMData & Get_SfMData() const { return _sfm_data; }

  bool Colorize() { return ColorizeTracks(_sfm_data); }

protected:
  std::string _sOutDirectory; // Output path where outputs will be stored

  //-----
  //-- Reconstruction data
  //-----
  SfMData _sfm_data; // internal SfMData

  //-----
  //-- Reconstruction parameters
  //-----
  bool _bFixedIntrinsics;
};

} // namespace sfm
} // namespace aliceVision
