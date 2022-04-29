// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IntrinsicsScaleOffset.hpp"
#include "Distortion.hpp"
#include <memory>

namespace aliceVision {
namespace camera {

/**
 * @brief Class with disto
 */
class IntrinsicsScaleOffsetDisto : public IntrinsicsScaleOffset
{
public:
  IntrinsicsScaleOffsetDisto() = default;

  IntrinsicsScaleOffsetDisto(unsigned int w, unsigned int h, double scaleX, double scaleY, double offsetX, double offsetY, std::shared_ptr<Distortion> distortion = nullptr)
  : IntrinsicsScaleOffset(w, h, scaleX, scaleY, offsetX, offsetY)
  , _pDistortion(distortion)
  {
    
  }

  void assign(const IntrinsicBase& other) override
  {
    *this = dynamic_cast<const IntrinsicsScaleOffsetDisto&>(other);
  }

  bool operator==(const IntrinsicBase& otherBase) const override
  {
      if(!IntrinsicsScaleOffset::operator==(otherBase))
          return false;
      if(typeid(*this) != typeid(otherBase))
          return false;
      const IntrinsicsScaleOffsetDisto& other = static_cast<const IntrinsicsScaleOffsetDisto&>(otherBase);

      if(_pDistortion != nullptr && other._pDistortion != nullptr)
          return (*_pDistortion) == (*other._pDistortion);
      return _pDistortion == other._pDistortion;
  }

  bool hasDistortion() const override
  {
    return _pDistortion != nullptr;
  }

  Vec2 addDistortion(const Vec2& p) const override
  {
    if (_pDistortion == nullptr)
    {
      return p;
    }
    return _pDistortion->addDistortion(p); 
  }
  

  Vec2 removeDistortion(const Vec2& p) const override
  {
    if (_pDistortion == nullptr)
    {
      return p;
    }
    return _pDistortion->removeDistortion(p); 
  }

  /// Return the un-distorted pixel (with removed distortion)
  Vec2 get_ud_pixel(const Vec2& p) const override
  {
    return cam2ima(toMeters(p));
  }

  /// Return the distorted pixel (with added distortion)
  Vec2 get_d_pixel(const Vec2& p) const override
  {
    return toPixels(ima2cam(p));
  }

  Vec2 toUnitless(const Vec2& p) const 
  {
      Vec2 pt;

      double hw = 0.5 * double(_w);
      double hh = 0.5 * double(_h);
      double diag = sqrt(hw * hw + hh * hh); 

      pt.x() = p.x() / diag;
      pt.y() = p.y() / diag;

      return p;
  }

  Eigen::Matrix<double, 2, 2> getDerivativeToUnitlessWrtPt(const Vec2 &p) const
  {
    double hw = 0.5 * double(_w);
    double hh = 0.5 * double(_h);
    double diag = sqrt(hw * hw + hh * hh); 

    Eigen::Matrix<double, 2, 2> ret;
    ret(0, 0) = 1.0 / diag;
    ret(0, 1) = 0;
    ret(0, 1) = 0;
    ret(1, 1) = 1.0 / diag;

    return ret;
  }

  Vec2 fromUnitless(const Vec2& p) const
  {
      Vec2 pt;

      double hw = 0.5 * double(_w);
      double hh = 0.5 * double(_h);
      double diag = sqrt(hw * hw + hh * hh);

      pt.x() = p.x() * diag;
      pt.y() = p.y() * diag;

      return p;
  }

  Eigen::Matrix<double, 2, 2> getDerivativeFromUnitlessWrtPt(const Vec2 &p) const
  {
    double hw = 0.5 * double(_w);
    double hh = 0.5 * double(_h);
    double diag = sqrt(hw * hw + hh * hh); 

    Eigen::Matrix<double, 2, 2> ret;
    ret(0, 0) = diag;
    ret(0, 1) = 0;
    ret(0, 1) = 0;
    ret(1, 1) = diag;

    return ret;
  }

  Vec2 toPixels(const Vec2 & p) const 
  { 
      if(!_useUnitlessDistortion)
      {
          return cam2ima(addDistortion(p));
      }
      

      const Vec2 pixCentered = cam2imaCentered(p);
      const Vec2 pixCenteredDisto = pixCentered + _distortionOffset;
      const Vec2 unitLess = toUnitless(pixCenteredDisto);
      const Vec2 distorted = addDistortion(unitLess);
      const Vec2 pixels = fromUnitless(distorted) + getPrincipalPoint();

      return pixels;
  }
  

  Eigen::Matrix<double, 2, 2> getDerivativeToPixelsWrtPt(const Vec2 & p) const
  { 
      if (!_useUnitlessDistortion)
      {
        return getDerivativeCam2ImaWrtPt() * getDerivativeAddDistoWrtPt(p);
      }
      else 
      {
        double hw = 0.5 * double(_w);
        double hh = 0.5 * double(_h);
        double diag = sqrt(hw * hw + hh * hh);
        
        const Vec2 pixCentered = cam2imaCentered(p);
        const Vec2 pixCenteredDisto = pixCentered + _distortionOffset;
        const Vec2 unitLess = toUnitless(pixCenteredDisto);
        const Vec2 distorted = addDistortion(unitLess);
        const Vec2 pixels = fromUnitless(distorted) + getPrincipalPoint();

        return getDerivativeFromUnitlessWrtPt(distorted) * getDerivativeAddDistoWrtPt(unitLess) * getDerivativeToUnitlessWrtPt(pixCenteredDisto) * getDerivativeCam2ImaWrtPt();
      }
  }

  Eigen::Matrix<double, 2, 2> getDerivativeToPixelsWrtScale(const Vec2& p) const
  {
      if (!_useUnitlessDistortion)
      {
        return getDerivativeCam2ImaWrtScale(p);
      }
      else 
      {
        double hw = 0.5 * double(_w);
        double hh = 0.5 * double(_h);
        double diag = sqrt(hw * hw + hh * hh);

        const Vec2 pixCentered = cam2imaCentered(p);
        const Vec2 pixCenteredDisto = pixCentered + _distortionOffset;
        const Vec2 unitLess = toUnitless(pixCenteredDisto);
        const Vec2 distorted = addDistortion(unitLess);
        const Vec2 pixels = fromUnitless(distorted) + getPrincipalPoint();

        return getDerivativeFromUnitlessWrtPt(distorted) * getDerivativeAddDistoWrtPt(unitLess) * getDerivativeToUnitlessWrtPt(pixCenteredDisto) * getDerivativeCam2ImaWrtScale(p);
      }
  }

  Eigen::Matrix<double, 2, 2> getDerivativeToPixelsWrtPrincipalPt(const Vec2& p) const
  {
      if (!_useUnitlessDistortion)
      {
        return getDerivativeCam2ImaWrtPrincipalPt();
      }
      else 
      {
        double hw = 0.5 * double(_w);
        double hh = 0.5 * double(_h);
        double diag = sqrt(hw * hw + hh * hh);

        const Vec2 pixCentered = cam2imaCentered(p);
        const Vec2 pixCenteredDisto = pixCentered + _distortionOffset;
        const Vec2 unitLess = toUnitless(pixCenteredDisto);
        const Vec2 distorted = addDistortion(unitLess);
        const Vec2 pixels = fromUnitless(distorted) + getPrincipalPoint();

        return Eigen::Matrix2d::Identity();
      }
  }

  Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeToPixelsWrtDisto(const Vec2 & p) const
  {
      if (!_useUnitlessDistortion)
      {
        return getDerivativeCam2ImaWrtPt() * getDerivativeAddDistoWrtDisto(p);
      }
      else 
      {
        double hw = 0.5 * double(_w);
        double hh = 0.5 * double(_h);
        double diag = sqrt(hw * hw + hh * hh);

        const Vec2 pixCentered = cam2imaCentered(p);
        const Vec2 pixCenteredDisto = pixCentered + _distortionOffset;
        const Vec2 unitLess = toUnitless(pixCenteredDisto);

        return getDerivativeFromUnitlessWrtPt(pixCenteredDisto) * getDerivativeAddDistoWrtDisto(unitLess);
      }
  }

  Eigen::Matrix2d getDerivativeToPixelsWrtOffset(const Vec2 & p) const 
  { 
      if (!_useUnitlessDistortion)
      {
        return getDerivativeCam2ImaWrtPrincipalPt();
      }
      else 
      {
        return Eigen::Matrix2d::Identity();
      }
  }

  Eigen::Matrix2d getDerivativeToPixelsWrtDistortionOffset(const Vec2 & p) const 
  { 
      if (!_useUnitlessDistortion)
      {
        return Eigen::Matrix2d::Zero();
      }
      else 
      {
        double hw = 0.5 * double(_w);
        double hh = 0.5 * double(_h);
        double diag = sqrt(hw * hw + hh * hh);

        const Vec2 pixCentered = cam2imaCentered(p);
        const Vec2 pixCenteredDisto = pixCentered + _distortionOffset;
        const Vec2 unitLess = toUnitless(pixCenteredDisto);
        const Vec2 distorted = addDistortion(unitLess);
        const Vec2 pixels = fromUnitless(distorted) + getPrincipalPoint();

        return getDerivativeFromUnitlessWrtPt(distorted) * getDerivativeAddDistoWrtPt(unitLess) * getDerivativeToUnitlessWrtPt(pixCenteredDisto);
      }
  }

  Vec2 toMeters(const Vec2 & p) const 
  { 
      if(!_useUnitlessDistortion)
      {
          return ima2cam(removeDistortion(p));
      }

      const Vec2 unitless = toUnitless(p - getPrincipalPoint());
      const Vec2 undistorted = removeDistortion(unitless);
      const Vec2 centered = fromUnitless(undistorted);
      const Vec2 centered2 = centered - _distortionOffset;
      const Vec2 meters = ima2camCentered(centered2);

      return meters;
  }

  Eigen::Matrix<double, 2, 2> getDerivativeToMetersWrtPt(const Vec2 & p) const
  { 
      if (!_useUnitlessDistortion)
      {
        return getDerivativeIma2CamWrtPt() * getDerivativeRemoveDistoWrtPt(p);
      }
      else 
      {
        const Vec2 unitless = toUnitless(p - getPrincipalPoint());
        const Vec2 undistorted = removeDistortion(unitless);
        const Vec2 centered = fromUnitless(undistorted);
        const Vec2 centered2 = centered - _distortionOffset;
        const Vec2 meters = ima2camCentered(centered2);

        return getDerivativeIma2CamWrtPt() * getDerivativeFromUnitlessWrtPt(undistorted) * getDerivativeRemoveDistoWrtPt(unitless) * getDerivativeToUnitlessWrtPt(p);
      }
  }

  Eigen::Matrix<double, 2, 2> getDerivativeToMetersWrtPrincipalPoint(const Vec2 & p) const
  { 
      if (!_useUnitlessDistortion)
      {
        return getDerivativeIma2CamWrtPrincipalPt();
      }
      else 
      {
        const Vec2 unitless = toUnitless(p - getPrincipalPoint());
        const Vec2 undistorted = removeDistortion(unitless);
        const Vec2 centered = fromUnitless(undistorted);
        const Vec2 centered2 = centered - _distortionOffset;
        const Vec2 meters = ima2camCentered(centered2);

        return getDerivativeIma2CamWrtPt() * getDerivativeFromUnitlessWrtPt(undistorted) * getDerivativeRemoveDistoWrtPt(unitless) * getDerivativeToUnitlessWrtPt(p);
      }
  }

  Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeToMetersWrtDisto(const Vec2 & p) const
  {
      if (!_useUnitlessDistortion)
      {
        return getDerivativeIma2CamWrtPt() * getDerivativeRemoveDistoWrtDisto(p);
      }
      else 
      {
        const Vec2 unitless = toUnitless(p - getPrincipalPoint());
        const Vec2 undistorted = removeDistortion(unitless);
        const Vec2 centered = fromUnitless(undistorted);
        const Vec2 centered2 = centered - _distortionOffset;
        const Vec2 meters = ima2camCentered(centered2);

        return getDerivativeIma2CamWrtPt() * getDerivativeFromUnitlessWrtPt(undistorted) * getDerivativeRemoveDistoWrtDisto(unitless);
      }
  }
  
  
  std::vector<double> getDistortionParams() const
  {
    if (!hasDistortion()) {
      return std::vector<double>();
    }

    return _pDistortion->getParameters();
  }

  void setDistortionParams(const std::vector<double> & distortionParams)
  {
    int expected = 0;
    if (_pDistortion != nullptr)
    {
      expected = _pDistortion->getDistortionParametersCount();
    }

    if (distortionParams.size() != expected)
    {
        std::stringstream s;
        s << "IntrinsicsScaleOffsetDisto::setDistortionParams: wrong number of distortion parameters (expected: " << expected << ", given:" << distortionParams.size() << ").";
        throw std::runtime_error(s.str());
    }

    if (_pDistortion)
    {
      _pDistortion->getParameters() = distortionParams;
    }
  }

  // Data wrapper for non linear optimization (get data)
  std::vector<double> getParams() const override
  {
    std::vector<double> params = {_scale(0), _scale(1), _offset(0), _offset(1), _distortionOffset(0), _distortionOffset(1)};

    if (hasDistortion())
    {
      params.insert(params.end(), _pDistortion->getParameters().begin(), _pDistortion->getParameters().end());
    }

    return params;
  }

  virtual bool updateParamsFromVersion(std::vector<double>& updatedParams, const std::vector<double>& params, const Version & inputVersion) const override 
  {
    if (!IntrinsicsScaleOffset::updateParamsFromVersion(updatedParams, params, inputVersion))
    {
      return false;
    }

    std::vector<double> localParams(updatedParams.size() + 2);

    localParams[0] = updatedParams[0];
    localParams[1] = updatedParams[1];
    localParams[2] = updatedParams[2];
    localParams[3] = updatedParams[3];
    localParams[4] = 0.0;
    localParams[5] = 0.0;

    for (int i = 4; i <= updatedParams.size(); i++)
    {
      localParams[i + 2] = updatedParams[i];
    }

    updatedParams = localParams;

    return true;
  }

  // Data wrapper for non linear optimization (update from data)
  virtual bool updateFromParams(const std::vector<double>& params) override
  {
    if (_pDistortion == nullptr)
    {
      if (params.size() != 6)
      {
        return false;
      }
    }
    else
    {
      if (params.size() != (6 + _pDistortion->getDistortionParametersCount()))
      {
        return false;
      }
    }

    _scale(0) = params[0];
    _scale(1) = params[1];
    _offset(0) = params[2];
    _offset(1) = params[3];
    _distortionOffset(0) = params[4];
    _distortionOffset(1) = params[5];

    setDistortionParams({params.begin() + 6, params.end()});

    return true;
  }

  float getMaximalDistortion(double min_radius, double max_radius) const override
  {
    if (_pDistortion == nullptr)
    {
      return max_radius;
    }

    return _pDistortion->getUndistortedRadius(max_radius);
  }

  Eigen::Matrix<double, 2, 2> getDerivativeAddDistoWrtPt(const Vec2 & pt) const
  {
    if (this->_pDistortion == nullptr)
    {
      return Eigen::Matrix<double, 2, 2>::Identity();
    }
    return this->_pDistortion->getDerivativeAddDistoWrtPt(pt);
  }

  Eigen::Matrix<double, 2, 2> getDerivativeRemoveDistoWrtPt(const Vec2 & pt) const
  {
    if (this->_pDistortion == nullptr)
    {
      return Eigen::Matrix<double, 2, 2>::Identity();
    }

    return this->_pDistortion->getDerivativeRemoveDistoWrtPt(pt);
  }

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & pt) const
  {
    if (this->_pDistortion == nullptr)
    {
      return Eigen::MatrixXd(0, 0);
    }

    return this->_pDistortion->getDerivativeAddDistoWrtDisto(pt);
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & pt) const
  {
    if (this->_pDistortion == nullptr)
    {
      return Eigen::MatrixXd(0, 0);
    }

    return this->_pDistortion->getDerivativeRemoveDistoWrtDisto(pt);
  }

  std::shared_ptr<Distortion> getDistortion() const
  {
      return _pDistortion;
  }

  ~IntrinsicsScaleOffsetDisto() override = default;

  bool useUnitlessDistortion() const 
  { 
      return _useUnitlessDistortion;
  }

  void useUnitlessDistortion(bool val)
  { 
      _useUnitlessDistortion = val;
  }

  Vec2 getDistortionOffset() const
  { 
      return _distortionOffset;
  }

  void setDistortionOffset(const Vec2 & val) 
  { 
      _distortionOffset = val;
  }

protected:
  std::shared_ptr<Distortion> _pDistortion;
  Vec2 _distortionOffset{0.0, 0.0};
  bool _useUnitlessDistortion{true};
};

} // namespace camera
} // namespace aliceVision
