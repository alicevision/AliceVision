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

#include <map>

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

    /**
    * @brief Comparison operator
    * @return true if both featureId and coordinates are equal in both observations
    */
    bool operator==(const Observation& other) const;

    /**
    * @brief Return the coordinates of the observation in pixels
    * @return The coordinates in pixels
    */
    const Vec2& getCoordinates() const { return _coordinates; }

    /**
    * @brief Return the coordinates of the observation in pixels
    * @return The coordinates in pixels
    */
    Vec2& getCoordinates() { return _coordinates; }

    /**
    * @brief Return the X coordinate of the observation in pixels
    * @return The column coordinates in pixels
    */
    double getX() const { return _coordinates.x(); }

    /**
    * @brief Return the Y coordinate of the observation in pixels
    * @return The row coordinates in pixels
    */
    double getY() const { return _coordinates.y(); }

    /**
    * @brief Set the image coordinates of the observation
    * @param coordinates the 2d vector with pixel coordinates
    */
    void setCoordinates(const Vec2& coordinates) { _coordinates = coordinates; }

    /**
    * @brief Set the image coordinates of the observation
    * @param x the x coordinates in the image (column)
    * @param y the x coordinates in the image (row)
    */
    void setCoordinates(double x, double y)
    {
        _coordinates(0) = x;
        _coordinates(1) = y;
    }

    /**
    * @brief An observation should be associated to a feature. This is the id of the feature.
    * @return featureId an id of the feature (view specific)
    */
    IndexT getFeatureId() const { return _idFeature; }

    /**
    * @brief An observation should be associated to a feature. This is the id of the feature.
    * @param featureId an id of the feature (view specific)
    */
    void setFeatureId(IndexT featureId) { _idFeature = featureId; }

    /**
    * @brief Get the measured scale of the feature (Scale of the source image)
    * @return the scale by which the image has been zoomed or 0 if unknown
    */
    double getScale() const { return _scale; }

    /**
    * @brief Set the measured scale of the feature (Scale of the source image)
    * @param scale the scale by which the image has been zoomed or 0 if unknown
    */
    void setScale(double scale) { _scale = scale; }

    /**
    * @brief Get the measured depth (Depth meaning is camera type dependent)
    * @return The optional measured depth, or less than 0 if non used
    */
    double getDepth() const { return _depth; }

    /**
    * @brief Set the measured point depth (Depth meaning is camera type dependent)
    * @param depth the point depth 
    */
    void setDepth(double depth) { _depth = depth; }


  private:
    Vec2 _coordinates = { 0.0, 0.0 };
    IndexT _idFeature = UndefinedIndexT;
    double _scale = 0.0;

    //Optional measured 'depth'
    double _depth = -1.0;
};

/// Observations are indexed by their View_id
typedef stl::flat_map<IndexT, Observation> Observations;
typedef std::map<IndexT, Observation> MapObservations;

}  // namespace sfmData
}  // namespace aliceVision
