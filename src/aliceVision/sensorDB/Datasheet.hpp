// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {
namespace sensorDB {

/**
 * @brief The Database structure
 */
struct Datasheet
{
  Datasheet() = default;

  /**
   * @brief Datasheet Constructor
   * @param brand
   * @param model
   * @param sensorSize
   */
  Datasheet( const std::string& brand,
             const std::string& model,
             const double& sensorSize ):
                  _brand(brand),
                  _model(model),
                  _sensorSize(sensorSize)
  {}

  bool operator==(const Datasheet& ds) const;

  std::string _brand;
  std::string _model;
  double _sensorSize{};
};

} // namespace sensorDB
} // namespace aliceVision
