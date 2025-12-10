// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/stl/FlatMap.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/Observation.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief Landmark is a 3D point with its 2d observations.
 */
class Landmark
{
  public:
    /**
     * @brief Default constructor
     */
    Landmark() {}

    /**
     * @brief Constructor with image describer type
     * @param descType The image describer type for this landmark
     */
    explicit Landmark(feature::EImageDescriberType descType)
      : _descType(descType)
    {}

    /**
     * @brief Constructor with 3D position, image describer type, and color
     * @param pos3d The 3D position of the landmark
     * @param descType The image describer type for this landmark
     * @param color The RGB color associated with the landmark
     */
    Landmark(const Vec3& pos3d,
             feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED,
             const image::RGBColor& color = image::WHITE)
      : _X(pos3d),
        _descType(descType),
        _rgb(color)
    {}

    /**
     * @brief Get the reference view index.
     * 
     * The reference view index identifies the view that serves as the reference frame
     * for this landmark's 3D position when using relative pose parameterization.
     * When set to UndefinedIndexT, the landmark uses absolute world coordinates.
     * 
     * @return The index of the reference view, or UndefinedIndexT if not set
     */
    IndexT getReferenceViewIndex() const { return _referenceViewIndex; }

    /**
     * @brief Set the reference view index.
     * 
     * Sets the view that serves as the reference frame for this landmark's 3D position.
     * This is used in relative pose parameterization during bundle adjustment.
     * Set to UndefinedIndexT to use absolute world coordinates.
     * 
     * @param viewIndex The index of the reference view, or UndefinedIndexT for absolute coordinates
     */
    void setReferenceViewIndex(IndexT viewIndex) { _referenceViewIndex = viewIndex; }

    /**
     * @brief Equality operator
     * @param other The landmark to compare with
     * @return true if landmarks are equal, false otherwise
     */
    bool operator==(const Landmark& other) const
    {
        return AreVecNearEqual(_X, other._X, 1e-3) 
        && AreVecNearEqual(_rgb, other._rgb, 1e-3) 
        && _observations == other._observations 
        && _descType == other._descType
        && _referenceViewIndex == other._referenceViewIndex;
    }

    /**
     * @brief Inequality operator
     * @param other The landmark to compare with
     * @return true if landmarks are not equal, false otherwise
     */
    inline bool operator!=(const Landmark& other) const { return !(*this == other); }

    /**
     * @brief Get the 3D position of the landmark (const version)
     * @return Const reference to the 3D position vector
     */
    const Vec3& getX() const { return _X; }

    /**
     * @brief Get the 3D position of the landmark (non-const version)
     * @return Reference to the 3D position vector
     */
    Vec3& getX() { return _X; }

    /**
     * @brief Set the 3D position of the landmark
     * @param X The new 3D position vector
     */
    void setX(const Vec3& X) { _X = X; }

    /**
     * @brief Get the observations (const version)
     * @return Const reference to the observations
     */
    const Observations& getObservations() const { return _observations; }

    /**
     * @brief Get the observations (non-const version)
     * @return Reference to the observations
     */
    Observations& getObservations() { return _observations; }

    /**
     * @brief Get the estimator parameter state of the landmark
     * @return The current state of the landmark
     */
    EEstimatorParameterState getState() const { return _state; }

    /**
     * @brief Set the estimator parameter state of the landmark
     * @param state The new state for the landmark
     */
    void setState(EEstimatorParameterState state) { _state = state; }

    /**
     * @brief Get the image describer type of the landmark
     * @return The image describer type
     */
    feature::EImageDescriberType getDescType() const { return _descType; }

    /**
     * @brief Set the image describer type of the landmark
     * @param descType The new image describer type
     */
    void setDescType(feature::EImageDescriberType descType) { _descType = descType; }

    /**
     * @brief Get the RGB color of the landmark
     * @return Const reference to the RGB color associated with the landmark
     */
    const image::RGBColor& getRgb() const { return _rgb; }

    /**
     * @brief Get the RGB color of the landmark (non-const version)
     * @return Reference to the RGB color associated with the landmark
     */
    image::RGBColor& getRgb() { return _rgb; }

    /**
     * @brief Set the RGB color of the landmark
     * @param rgb The new RGB color for the landmark
     */
    void setRgb(const image::RGBColor& rgb) { _rgb = rgb; }

    /**
     * @brief Get observations as a map
     * @return Map of observations
     */
    MapObservations getMapObservations() const 
    { 
        return MapObservations(_observations.begin(), _observations.end());;
    }
    
    /**
     * @brief Update the 3D position from estimator data
     * @param data Array containing the new 3D coordinates [x, y, z]
     * @note The landmark will only be updated if its state is REFINED
     */
    inline void updateFromEstimator(const std::array<double, 3> & data)
    {
        // do not update a landmark set as Ignored or Constant in the Local strategy
        if (_state != EEstimatorParameterState::REFINED)
        {
            return;
        }

        for (std::size_t i = 0; i < 3; ++i)
        {
            _X(Eigen::Index(i)) = data.at(i);
        }
    }

    /**
     * @brief Is this landmark robustness independent of parallax
     * @return true if this landmark has this special property
    */
    bool isParallaxRobust() const { return _parallaxRobust; }

    /**
    * @brief decide if this landmark is robust even if its parallax is low
    * @param parallaxRobust True if robust to lack of parallax
    */
    void setParallaxRobust(bool parallaxRobust) { _parallaxRobust = parallaxRobust; }

    /**
     * @brief Is this landmark precisely located
     * @return true if this landmark has this special property
    */
    bool isPrecise() const { return _isPrecise; }

    /**
    * @brief decide if this landmark is robust even if its parallax is low
    * @param isPrecise True if robust to lack of parallax
    */
    void setIsPrecise(bool isPrecise) { _isPrecise = isPrecise; }

private:
    Vec3 _X = { 0.0, 0.0, 0.0 };  //!> the 3D position of the landmark
    Observations _observations;
    EEstimatorParameterState _state = EEstimatorParameterState::REFINED;
    feature::EImageDescriberType _descType = feature::EImageDescriberType::UNINITIALIZED;
    bool _parallaxRobust = false;
    bool _isPrecise = false;
    image::RGBColor _rgb = image::WHITE;  //!> the color associated to the point
    IndexT _referenceViewIndex = UndefinedIndexT;  //!> the index of the reference view for relative pose parameterization
};

}  // namespace sfmData
}  // namespace aliceVision
