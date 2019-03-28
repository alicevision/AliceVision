// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#pragma once
#include <cstddef>
#include <cassert>
#include <cmath>
#include <vector>
#include <array>
#include <iostream>


namespace aliceVision {
namespace hdr {
  
class rgbCurve
{
public:

  /**
   * @brief rgbCurve constructor
   * @param[in] size - size of each curve
   */
  rgbCurve(std::size_t size);
  
  /**
   * @brief rgbCurve constructor
   * @param[in] path - filepath of an rgbCurve file
   */
  rgbCurve(const std::string &path);
   
  /**
   * @brief set curves to zero
   */
  void setZero()
  {
    for(auto &curve : _data)
    {
      std::fill(curve.begin(), curve.end(), 0.0f);
    } 
  }

  /**
   * @brief set curves to one
   */
  void setOne()
  {
    for(auto &curve : _data)
    {
      std::fill(curve.begin(), curve.end(), 1.0f);
    } 
  }

  /**
   * @brief set a value at an index for each curves
   * @param index
   * @param value
   */
  void setAllChannels(std::size_t index , float value)
  {
    assert(index < getSize());

    for(auto &curve : _data)
    {
      curve[index] = value;
    }
  }

  /**
   * @brief Set curves to linear
   */
  void setLinear();
  
  /**
   * @brief Set curves to linear
   */
  void setGamma();
  
  /**
   *@brief Set curves to gaussian
   */
  void setGaussian(double size = 1);
  
  /**
   *@brief Set curves to triangular
   */
  void setTriangular();
  
  /**
   *@brief Set curves to plateau
   */
  void setPlateau();
  
  /**
   *@brief Set curves to plateau
   */
  void setLog10();
  
  /**
   * @brief inverse all values of the image
   * rgbCurve(i) = 1/i
   */
  void inverseAllValues();
  
  /**
   * @brief change all value of the image by their absolute value
   */
  void setAllAbsolute();

  /*
   *@brief normalize the curve
   */
  void normalize();
  
  /**
   * @brief interpolates all values at zero with the previous an the next value 
   */
  void interpolateMissingValues();

  /**
   * @brief Left accessor
   * @param[in] index
   * @param[in] channel
   * @return the value at the index of the channel curve
   */
  float& operator() (float sample, std::size_t channel);


  /**
   * @brief Right accessor
   * @param[in] sample
   * @param[in] channel
   * @return the value at the index of the channel curve
   */
  float operator() (float sample , std::size_t channel) const;
  
  /**
   * @brief Operator+ Call sum method
   * @param[in] other
   * @return rgbCurve of the sum
   */
  const rgbCurve operator+(const rgbCurve &other) const;

  /**
   * @brief Operator- Call subtract method
   * @param[in] other
   * @return rgbCurve of the subtraction
   */
  const rgbCurve operator-(const rgbCurve &other) const; 
  
  /**
   * @brief Operator*= Call multiply method
   * @param[in] other - rgbCurve
   */
  void operator*=(const rgbCurve &other);

  /**
   * @brief Sum all value of an rgbCurve by another of the same size
   * @param[in] other
   * @return rgbCurve of the sum
   */
  const rgbCurve sum(const rgbCurve &other) const;

  /**
   * @brief Subtract all value of an rgbCurve by another of the same size
   * @param[in] other
   * @return rgbCurve of the subtraction
   */
  const rgbCurve subtract(const rgbCurve &other) const;
  
  /**
   * @brief Multiply all value of an rgbCurve by another of the same size
   * @param[in] other
   */
  void multiply(const rgbCurve &other);

  /**
   * @brief Write in a csv file 
   * @param[in] path
   */
  void write(const std::string &path, const std::string &name = "rgbCurve") const;
  
  /**
   * @brief Read and fill curves from a csv file 
   * @param[in] path
   */
  void read(const std::string &path);

  bool isEmpty() const
  {
    return _data.front().empty();
  }

  std::size_t getIndex(float sample) const
  {
    assert(getSize() != 0);
    if(sample < 0.0f)
      return 0;
    if(sample > 1.0f)
      return getSize() - 1;
    return std::size_t(std::round(sample * (getSize() - 1)));
  }

  std::size_t getSize() const
  {
    return _data.front().size();
  }
 
  std::size_t getNbChannels() const
  {
    return _data.size();
  }

  const std::vector<float>& getCurve(std::size_t channel) const
  {
    assert(channel < 3);
    return _data[channel];
  }

  std::vector<float>& getCurve(std::size_t channel)
  {
    assert(channel < 3);
    return _data[channel];
  }
  
  const std::vector<float>& getCurveRed() const
  {
    return _data[0];
  }

  std::vector<float>& getCurveRed()
  {
    return _data[0];
  }
  
  const std::vector<float>& getCurveGreen() const
  {
    return _data[1];
  }

  std::vector<float>& getCurveGreen()
  {
    return _data[1];
  }
  
  const std::vector<float>& getCurveBlue() const
  {
    return _data[2];
  }

  std::vector<float>& getCurveBlue()
  {
    return _data[2];
  }

  /**
   * @brief Sum of all value of all channel
   * @param[in] curve
   * @return the sum scalar
   */
  static double sumAll(const rgbCurve &curve);

private: 
  std::array< std::vector<float>, 3 > _data;
};
    
} // namespace hdr
} // namespace aliceVision
