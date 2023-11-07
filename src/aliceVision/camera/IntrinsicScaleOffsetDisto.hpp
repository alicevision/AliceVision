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

    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const IntrinsicScaleOffsetDisto&>(other); }

    bool operator==(const IntrinsicBase& otherBase) const override;

    void setDistortionObject(std::shared_ptr<Distortion> object) { _pDistortion = object; }

    bool hasDistortion() const override { return _pDistortion != nullptr || _pUndistortion != nullptr; }

    /**
     * @brief Create a new point from a given point by adding distortion.
     * @param[in] p Point in the camera plane.
     * @return Distorted point in the camera plane.
     */
    Vec2 addDistortion(const Vec2& p) const override
    {
        if (_pDistortion)
        {
            return _pDistortion->addDistortion(p);
        }
        else if (_pUndistortion)
        {
            return ima2cam(_pUndistortion->inverse(cam2ima(p)));
        }
        return p;
    }

    /**
     * @brief Create a new point from a given point by removing distortion.
     * @param[in] p Point in the camera plane.
     * @return Undistorted point in the camera plane.
     */
    Vec2 removeDistortion(const Vec2& p) const override
    {
        if (_pUndistortion)
        {
            return ima2cam(_pUndistortion->undistort(cam2ima(p)));
        }
        else if (_pDistortion)
        {
            return _pDistortion->removeDistortion(p);
        }
        return p;
    }

    /// Return the un-distorted pixel (with removed distortion)
    Vec2 get_ud_pixel(const Vec2& p) const override;

    /// Return the distorted pixel (with added distortion)
    Vec2 get_d_pixel(const Vec2& p) const override;

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
        int expected = 0;
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
    std::vector<double> getParams() const override
    {
        std::vector<double> params = {_scale(0), _scale(1), _offset(0), _offset(1)};

        if (_pDistortion)
        {
            params.insert(params.end(), _pDistortion->getParameters().begin(), _pDistortion->getParameters().end());
        }

        return params;
    }

    std::size_t getParamsSize() const override
    {
        std::size_t size = 4;
        if (_pDistortion)
        {
            size += _pDistortion->getParameters().size();
        }
        return size;
    }

    // Data wrapper for non linear optimization (update from data)
    bool updateFromParams(const std::vector<double>& params) override;

    float getMaximalDistortion(double min_radius, double max_radius) const override
    {
        if (_pDistortion == nullptr)
        {
            return max_radius;
        }

        return _pDistortion->getUndistortedRadius(max_radius);
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
