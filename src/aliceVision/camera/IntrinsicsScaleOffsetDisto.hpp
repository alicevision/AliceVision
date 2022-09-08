// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IntrinsicsScaleOffset.hpp"
#include "IntrinsicInitMode.hpp"
#include "Distortion.hpp"
#include "Undistortion.hpp"
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

  IntrinsicsScaleOffsetDisto(unsigned int w, unsigned int h, double scaleX, double scaleY, double offsetX, double offsetY, std::shared_ptr<Distortion> distortion = nullptr, EInitMode distortionInitializationMode = EInitMode::NONE)
  : IntrinsicsScaleOffset(w, h, scaleX, scaleY, offsetX, offsetY)
  , _pDistortion(distortion)
  , _distortionInitializationMode(distortionInitializationMode)
  {}

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

      if (_distortionInitializationMode != other._distortionInitializationMode)
          return false;

      if(_pDistortion != nullptr && other._pDistortion != nullptr)
          return (*_pDistortion) == (*other._pDistortion);
      return _pDistortion == other._pDistortion;
  }

  void setDistortionObject(std::shared_ptr<Distortion> object)
  {
      _pDistortion = object;
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
    return cam2ima(removeDistortion(ima2cam(p)));
  }

  /// Return the distorted pixel (with added distortion)
  Vec2 get_d_pixel(const Vec2& p) const override
  {
    return cam2ima(addDistortion(ima2cam(p)));
  }

  std::size_t getDistortionParamsSize() const
  {
    if (_pDistortion == nullptr)
        return 0;
    return _pDistortion->getParameters().size();
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

    if (hasDistortion())
    {
      params.insert(params.end(), _pDistortion->getParameters().begin(), _pDistortion->getParameters().end());
    }

    return params;
  }

  std::size_t getParamsSize() const override
  {
    std::size_t size = 4;
    if (hasDistortion())
    {
      size += _pDistortion->getParameters().size();
    }
    return size;
  }

  // Data wrapper for non linear optimization (update from data)
  bool updateFromParams(const std::vector<double>& params) override
  {
    if (_pDistortion == nullptr)
    {
      if (params.size() != 4)
      {
        return false;
      }
    }
    else
    {
      if (params.size() != (4 + _pDistortion->getDistortionParametersCount()))
      {
        return false;
      }
    }

    _scale(0) = params[0];
    _scale(1) = params[1];
    _offset(0) = params[2];
    _offset(1) = params[3];

    setDistortionParams({params.begin() + 4, params.end()});

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
  inline EInitMode getDistortionInitializationMode() const override
  {
      return _distortionInitializationMode;
  }

  ~IntrinsicsScaleOffsetDisto() override = default;

protected:
  void throwSetDistortionParamsCountError(std::size_t expected, std::size_t received)
  {
      std::stringstream s;
      s << "IntrinsicsScaleOffsetDisto::setDistortionParams*: "
        << "wrong number of distortion parameters (expected: "
        << expected << ", given:" << received << ").";
      throw std::runtime_error(s.str());
  }

  std::shared_ptr<Distortion> _pDistortion;

  // Distortion initialization mode
  EInitMode _distortionInitializationMode = EInitMode::NONE;
};

} // namespace camera
} // namespace aliceVision
