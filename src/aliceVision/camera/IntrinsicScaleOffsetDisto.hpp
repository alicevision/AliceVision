// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IntrinsicScaleOffset.hpp"
#include "IntrinsicInitMode.hpp"
#include "Distortion.hpp"
#include "Undistortion.hpp"

#include <memory>

namespace aliceVision {
namespace camera {

/**
 * @brief Class with disto
 */
class IntrinsicScaleOffsetDisto : public IntrinsicScaleOffset
{
  public:
    IntrinsicScaleOffsetDisto(unsigned int w,
                              unsigned int h,
                              double scaleX,
                              double scaleY,
                              double offsetX,
                              double offsetY,
                              std::shared_ptr<Distortion> distortion = nullptr,
                              std::shared_ptr<Undistortion> undistortion = nullptr)
      : IntrinsicScaleOffset(w, h, scaleX, scaleY, offsetX, offsetY),
        _pDistortion(distortion),
        _pUndistortion(undistortion)
    {}

    IntrinsicScaleOffsetDisto(const IntrinsicScaleOffsetDisto& other)
      : IntrinsicScaleOffset(other),
        _distortionInitializationMode(other._distortionInitializationMode)
    {
        if (other._pDistortion)
        {
            _pDistortion = std::shared_ptr<Distortion>(other._pDistortion->clone());
        }
        else
        {
            _pDistortion = nullptr;
        }

        if (other._pUndistortion)
        {
            _pUndistortion = std::shared_ptr<Undistortion>(other._pUndistortion->clone());
        }
        else
        {
            _pUndistortion = nullptr;
        }
    }

    // Deep copy of the (un)distortion, as the copy constructor does.
    IntrinsicScaleOffsetDisto& operator=(const IntrinsicScaleOffsetDisto& other)
    {
        if (this == &other)
        {
            return *this;
        }

        IntrinsicScaleOffset::operator=(other);
        _distortionInitializationMode = other._distortionInitializationMode;
        _pDistortion = other._pDistortion ? std::shared_ptr<Distortion>(other._pDistortion->clone()) : nullptr;
        _pUndistortion = other._pUndistortion ? std::shared_ptr<Undistortion>(other._pUndistortion->clone()) : nullptr;

        return *this;
    }

    static std::shared_ptr<IntrinsicScaleOffsetDisto> cast(std::shared_ptr<IntrinsicBase> sptr);

    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const IntrinsicScaleOffsetDisto&>(other); }

    /**
    * @brief compare to another intrinsic object
    * @param otherBase an intrinsic object to compare to.
    * @return true if both objects are considered equal
    */
    bool operator==(const IntrinsicBase& otherBase) const override;

    /**
    * @brief compare to another intrinsic object
    * @param otherBase an intrinsic object to compare to.
    * @param ignoreDistortion ignore difference in distortion.
    * @return true if both objects are considered equal
    */
    bool equalTo(const IntrinsicBase& otherBase, bool ignoreDistortion) const;

    void setDistortionObject(std::shared_ptr<Distortion> object) { _pDistortion = object; }

    bool hasDistortion() const override { return _pDistortion != nullptr || _pUndistortion != nullptr; }

    /**
     * @brief Create a new point from a given point by adding distortion.
     *        The distortion object is applied first, in the camera plane, then the inverse of the
     *        undistortion object, in pixels.
     * @param[in] p Point in the camera plane.
     * @return Distorted point in the camera plane.
     */
    Vec2 addDistortion(const Vec2& p) const override
    {
        Vec2 distorted = p;
        if (_pDistortion)
        {
            distorted = _pDistortion->addDistortion(distorted);
        }
        if (_pUndistortion)
        {
            distorted = ima2cam(_pUndistortion->inverse(cam2ima(distorted)));
        }
        return distorted;
    }

    /**
     * @brief Create a new point from a given point by removing distortion.
     *        The undistortion object is applied first, in pixels, then the inverse of the
     *        distortion object, in the camera plane.
     * @param[in] p Point in the camera plane.
     * @return Undistorted point in the camera plane.
     */
    Vec2 removeDistortion(const Vec2& p) const override
    {
        Vec2 undistorted = p;
        if (_pUndistortion)
        {
            undistorted = ima2cam(_pUndistortion->undistort(cam2ima(undistorted)));
        }
        if (_pDistortion)
        {
            undistorted = _pDistortion->removeDistortion(undistorted);
        }
        return undistorted;
    }

    /**
     * @brief Projection of a 3D point applying the Distortion object only (the Undistortion object is
     *        ignored), so the result lies in undistorted space. Equal to transformProject(pose, pt, true)
     *        when there is no Undistortion.
     *        This is the function differentiated by getDerivativeTransformProjectWrt*.
     * @param[in] pose camera pose
     * @param[in] pt homogeneous 3D point
     * @return undistorted pixel
     */
    virtual Vec2 transformProjectDistortionOnly(const Eigen::Matrix4d& pose, const Vec4& pt) const = 0;

    /**
     * @brief Back-projection of an undistorted pixel removing the Distortion object only (the Undistortion
     *        object is ignored). Equal to backProjectUnit(pt2D) when there is no Undistortion.
     *        This is the function differentiated by getDerivativeBackProjectUnitWrt*.
     * @param[in] pt2D undistorted pixel
     * @return unit ray in the camera frame
     */
    Vec3 backProjectUnitDistortionOnly(const Vec2& pt2D) const { return toUnitSphere(removeDistortionOnly(ima2cam(pt2D))); }

    /// Return the un-distorted pixel (with removed distortion)
    Vec2 getUndistortedPixel(const Vec2& p) const override;

    /// Return the distorted pixel (with added distortion)
    Vec2 getDistortedPixel(const Vec2& p) const override;

    std::size_t getDistortionParamsSize() const
    {
        if (_pDistortion)
        {
            return _pDistortion->getParameters().size();
        }
        return 0;
    }

    std::vector<double> getDistortionParams() const
    {
        if (!_pDistortion)
        {
            return std::vector<double>();
        }
        return _pDistortion->getParameters();
    }

    void setDistortionParams(const std::vector<double>& distortionParams)
    {
        std::size_t expected = 0;
        if (_pDistortion != nullptr)
        {
            expected = _pDistortion->getDistortionParametersCount();
        }

        if (distortionParams.size() != expected)
        {
            throwSetDistortionParamsCountError(expected, distortionParams.size());
        }

        if (_pDistortion)
        {
            _pDistortion->getParameters() = distortionParams;
        }
    }

    template<class F>
    void setDistortionParamsFn(F&& callback)
    {
        if (_pDistortion == nullptr)
            return;

        auto& params = _pDistortion->getParameters();
        for (std::size_t i = 0; i < params.size(); ++i)
        {
            params[i] = callback(i);
        }
    }

    template<class F>
    void setDistortionParamsFn(std::size_t count, F&& callback)
    {
        if (_pDistortion == nullptr)
        {
            if (count != 0)
            {
                throwSetDistortionParamsCountError(0, count);
            }
            return;
        }

        auto& params = _pDistortion->getParameters();
        if (params.size() != count)
        {
            throwSetDistortionParamsCountError(params.size(), count);
        }

        for (std::size_t i = 0; i < params.size(); ++i)
        {
            params[i] = callback(i);
        }
    }

    // Data wrapper for non linear optimization (get data)
    std::vector<double> getParameters() const override
    {
        std::vector<double> params = {_scale(0), _scale(1), _offset(0), _offset(1)};

        return params;
    }

    std::size_t getParametersSize() const override { return 4; }

    // Data wrapper for non linear optimization (update from data)
    bool updateFromParams(const std::vector<double>& params) override;

    /**
     * @brief Rescale the intrinsic, including the undistortion which is expressed in pixels.
     *        Uniform rescales and desqueezing (when the factors ratio cancels the pixel aspect ratio) are exact
     *        for every undistortion model. Other non uniform rescales are not exact for the anamorphic model.
     * @param[in] factorW horizontal scale factor
     * @param[in] factorH vertical scale factor
     */
    void rescale(float factorW, float factorH) override;

    float getMaximalDistortion([[maybe_unused]] double minRadius, double maxRadius) const override
    {
        if (_pDistortion == nullptr)
        {
            return maxRadius;
        }

        return _pDistortion->getUndistortedRadius(maxRadius);
    }

    Eigen::Matrix<double, 2, 2> getDerivativeAddDistoWrtPt(const Vec2& pt) const
    {
        if (this->_pDistortion == nullptr)
        {
            return Eigen::Matrix<double, 2, 2>::Identity();
        }
        return this->_pDistortion->getDerivativeAddDistoWrtPt(pt);
    }

    Eigen::Matrix<double, 2, 2> getDerivativeRemoveDistoWrtPt(const Vec2& pt) const
    {
        if (this->_pDistortion == nullptr)
        {
            return Eigen::Matrix<double, 2, 2>::Identity();
        }

        return this->_pDistortion->getDerivativeRemoveDistoWrtPt(pt);
    }

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& pt) const
    {
        if (this->_pDistortion == nullptr)
        {
            return Eigen::MatrixXd(0, 0);
        }

        return this->_pDistortion->getDerivativeAddDistoWrtDisto(pt);
    }

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& pt) const
    {
        if (this->_pDistortion == nullptr)
        {
            return Eigen::MatrixXd(0, 0);
        }

        return this->_pDistortion->getDerivativeRemoveDistoWrtDisto(pt);
    }

    virtual Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeTransformProjectWrtDistortion(const Eigen::Matrix4d& pose,
                                                                                                const Vec4& pt) const = 0;

    virtual Eigen::Matrix<double, 3, Eigen::Dynamic> getDerivativeBackProjectUnitWrtDistortion(const Vec2& pt2D) const = 0;

    /**
     * @brief Set The intrinsic disto initialization mode
     * @param[in] distortionInitializationMode The intrintrinsic distortion initialization mode enum
     */
    inline void setDistortionInitializationMode(EInitMode distortionInitializationMode) override
    {
        _distortionInitializationMode = distortionInitializationMode;
    }

    /**
     * @brief Get the intrinsic disto initialization mode
     * @return The intrinsic disto initialization mode
     */
    inline EInitMode getDistortionInitializationMode() const override { return _distortionInitializationMode; }

    std::shared_ptr<Distortion> getDistortion() const { return _pDistortion; }

    ~IntrinsicScaleOffsetDisto() override = default;

    void setUndistortionObject(std::shared_ptr<Undistortion> object) { _pUndistortion = object; }

    std::shared_ptr<Undistortion> getUndistortion() const { return _pUndistortion; }

  protected:
    /// Apply the Distortion object only (identity if there is none).
    Vec2 addDistortionOnly(const Vec2& p) const { return _pDistortion ? _pDistortion->addDistortion(p) : p; }

    /// Remove the Distortion object only (identity if there is none).
    Vec2 removeDistortionOnly(const Vec2& p) const { return _pDistortion ? _pDistortion->removeDistortion(p) : p; }

    void throwSetDistortionParamsCountError(std::size_t expected, std::size_t received)
    {
        std::stringstream s;
        s << "IntrinsicScaleOffsetDisto::setDistortionParams*: "
          << "wrong number of distortion parameters (expected: " << expected << ", given:" << received << ").";
        throw std::runtime_error(s.str());
    }

    std::shared_ptr<Distortion> _pDistortion;
    std::shared_ptr<Undistortion> _pUndistortion;

    // Distortion initialization mode
    EInitMode _distortionInitializationMode = EInitMode::NONE;
};

}  // namespace camera
}  // namespace aliceVision
