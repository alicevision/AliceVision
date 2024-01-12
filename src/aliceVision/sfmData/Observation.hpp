// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/stl/FlatMap.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief 2D observation of a 3D landmark.
 */
class Observation
{
  public:
    Observation() {}

    Observation(const Vec2& p, IndexT idFeat, double scale_)
      : _coordinates(p),
        _idFeature(idFeat),
        _scale(scale_)
    {}

    bool operator==(const Observation& other) const;

    const Vec2& getCoordinates() const { return _coordinates; }

    Vec2& getCoordinates() { return _coordinates; }

    double getX() const { return _coordinates.x(); }

    double getY() const { return _coordinates.y(); }

    void setCoordinates(const Vec2& coordinates) { _coordinates = coordinates; }

    void setCoordinates(double x, double y)
    {
        _coordinates(0) = x;
        _coordinates(1) = y;
    }

    IndexT getFeatureId() const { return _idFeature; }

    void setFeatureId(IndexT featureId) { _idFeature = featureId; }

    double getScale() const { return _scale; }

    void setScale(double scale) { _scale = scale; }

  private:
    Vec2 _coordinates;
    IndexT _idFeature = UndefinedIndexT;
    double _scale = 0.0;
};

/// Observations are indexed by their View_id
typedef stl::flat_map<IndexT, Observation> Observations;

}  // namespace sfmData
}  // namespace aliceVision
