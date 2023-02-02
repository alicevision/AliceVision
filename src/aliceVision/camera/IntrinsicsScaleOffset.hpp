// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IntrinsicBase.hpp"

#include <aliceVision/version.hpp>

namespace aliceVision {
namespace camera {

/**
 * @brief Class with "focal" (scale) and center offset
 */
class IntrinsicsScaleOffset: public IntrinsicBase
{
public:
    IntrinsicsScaleOffset() = default;

    IntrinsicsScaleOffset(unsigned int w, unsigned int h,
                          double scaleX, double scaleY,
                          double offsetX, double offsetY) :
        IntrinsicBase(w, h), 
        _scale(scaleX, scaleY), _offset(offsetX, offsetY)
    {
    }

    ~IntrinsicsScaleOffset() override = default;

    void copyFrom(const IntrinsicsScaleOffset& other) { *this = other; }

    bool operator==(const IntrinsicBase& otherBase) const override
    {
        if (!IntrinsicBase::operator==(otherBase))
            return false;
        if (typeid(*this) != typeid(otherBase))
            return false;
        const IntrinsicsScaleOffset& other = static_cast<const IntrinsicsScaleOffset&>(otherBase);
        return _scale.isApprox(other._scale) && _offset.isApprox(other._offset);
    }

    void setScale(const Vec2& scale) { _scale = scale; }

    inline Vec2 getScale() const { return _scale; }

    void setOffset(const Vec2& offset) { _offset = offset; }

    inline Vec2 getOffset() const { return _offset; }

    /**
     * @brief Principal point in image coordinate ((0,0) is image top-left).
     */
    inline const Vec2 getPrincipalPoint() const 
    {
        Vec2 ret = _offset;
        ret(0) += double(_w) * 0.5;
        ret(1) += double(_h) * 0.5;

        return ret;
    }

    // Transform a point from the camera plane to the image plane
    Vec2 cam2ima(const Vec2& p) const override
    {
        return p.cwiseProduct(_scale) + getPrincipalPoint();
    }

    virtual Eigen::Matrix2d getDerivativeCam2ImaWrtScale(const Vec2& p) const
    {
        Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

        M(0, 0) = p(0);
        M(1, 1) = p(1);
            
        return M;
    }

    virtual Eigen::Matrix2d getDerivativeCam2ImaWrtPoint() const
    {
        Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

        M(0, 0) = _scale(0);
        M(1, 1) = _scale(1);

        return M;
    }

    virtual Eigen::Matrix2d getDerivativeCam2ImaWrtPrincipalPoint() const
    {
        return Eigen::Matrix2d::Identity();
    }

    // Transform a point from the image plane to the camera plane
    Vec2 ima2cam(const Vec2& p) const override
    {
        Vec2 np;

        Vec2 pp = getPrincipalPoint();

        np(0) = (p(0) - pp(0)) / _scale(0);
        np(1) = (p(1) - pp(1)) / _scale(1);

        return np;
    }

    virtual Eigen::Matrix<double, 2, 2> getDerivativeIma2CamWrtScale(const Vec2& p) const
    {
        Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

        Vec2 pp = getPrincipalPoint();

        M(0, 0) = -(p(0) - pp(0)) / (_scale(0) * _scale(0));
        M(1, 1) = -(p(1) - pp(1)) / (_scale(1) * _scale(1));

        return M;
    }

    virtual Eigen::Matrix2d getDerivativeIma2CamWrtPoint() const
    {
        Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

        M(0, 0) = 1.0 / _scale(0);
        M(1, 1) = 1.0 / _scale(1);

        return M;
    }

    virtual Eigen::Matrix2d getDerivativeIma2CamWrtPrincipalPoint() const
    {
        Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

        M(0, 0) = - 1.0 / _scale(0);
        M(1, 1) = - 1.0 / _scale(1);

        return M;
    }

    /**
     * @brief Rescale intrinsics to reflect a rescale of the camera image
     * @param factor a scale factor
     */
    void rescale(float factor) override
    {
        IntrinsicBase::rescale(factor);

        _scale *= factor;
        _offset *= factor;
    }

    // Data wrapper for non linear optimization (update from data)
    bool updateFromParams(const std::vector<double>& params) override
    {
        if (params.size() != 4)
        {
            return false;
        }

        _scale(0) = params[0];
        _scale(1) = params[1];
        _offset(0) = params[2];
        _offset(1) = params[3];

        return true;
    }

    /**
     * @brief Import a vector of params loaded from a file. It is similar to updateFromParams but it deals with file compatibility.
     */
    bool importFromParams(const std::vector<double>& params, const Version & inputVersion) override
    {
        std::vector<double> paramsLocal;
        if (inputVersion < Version(1, 2, 0))
        {
            paramsLocal.resize(params.size() + 1);
            paramsLocal[0] = params[0];
            paramsLocal[1] = params[0];

            for (int i = 1; i < params.size(); i++)
            {
                paramsLocal[i + 1] = params[i];
            }
        }
        else
        {
            paramsLocal = params;
        }

        if (!updateFromParams(paramsLocal))
        {
            return false;
        }

        if (inputVersion < Version(1, 2, 1))
        {
            _offset(0) -= double(_w) / 2.0;
            _offset(1) -= double(_h) / 2.0;
        }

        return true;
    }

    /**
     * @brief Set initial Scale (for constraining minimization)
     */
    inline void setInitialScale(const Vec2 & initialScale) { _initialScale = initialScale; }

    /**
     * @brief Get the intrinsic initial scale
     * @return The intrinsic initial scale
     */
    inline Vec2 getInitialScale() const { return _initialScale; }

    /**
     * @brief lock the ratio between fx and fy
     * @param lock is the ratio locked
     */
    void setRatioLocked(bool lock) { _ratioLocked = lock; }

    bool isRatioLocked() const { return _ratioLocked; }

protected:
    Vec2 _scale{1.0, 1.0};
    Vec2 _offset{0.0, 0.0};
    Vec2 _initialScale{-1.0, -1.0};
    bool _ratioLocked{true};
};

} // namespace camera
} // namespace aliceVision
