// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cassert>
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include "emorCurve.hpp"


namespace aliceVision {
namespace hdr {

enum class EFunctionType
{
      LINEAR,
      GAUSSIAN,
      TRIANGLE,
      PLATEAU,
      GAMMA,
      LOG10
};

/**
 * @brief convert an enum EFunctionType to its corresponding string
 * @param EFunctionType
 * @return String
 */
inline std::string EFunctionType_enumToString(const EFunctionType functionType)
{
  switch(functionType)
  {
    case EFunctionType::LINEAR:     return "linear";
    case EFunctionType::GAUSSIAN:   return "gaussian";
    case EFunctionType::TRIANGLE:   return "triangle";
    case EFunctionType::PLATEAU:    return "plateau";
    case EFunctionType::GAMMA:      return "gamma";
    case EFunctionType::LOG10:      return "log10";
  }
  throw std::out_of_range("Invalid function type enum");
}

/**
 * @brief convert a string function type to its corresponding enum EFunctionType
 * @param String
 * @return EFunctionType
 */
inline EFunctionType EFunctionType_stringToEnum(const std::string& functionType)
{
  std::string type = functionType;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  if(type == "linear")     return EFunctionType::LINEAR;
  if(type == "gaussian")   return EFunctionType::GAUSSIAN;
  if(type == "triangle")   return EFunctionType::TRIANGLE;
  if(type == "plateau")    return EFunctionType::PLATEAU;
  if(type == "gamma")      return EFunctionType::GAMMA;
  if(type == "log10")      return EFunctionType::LOG10;

  throw std::out_of_range("Invalid function type : '" + functionType + "'");
}

inline std::ostream& operator<<(std::ostream& os, const EFunctionType functionType)
{
  os << EFunctionType_enumToString(functionType);
  return os;
}

inline std::istream& operator>>(std::istream& in, EFunctionType& functionType)
{
  std::string token;
  in >> token;
  functionType = EFunctionType_stringToEnum(token);
  return in;
}


class rgbCurve
{
public:

  /**
   * @brief rgbCurve constructor
   * @param[in] size - size of each curve
   */
  explicit rgbCurve(std::size_t size);

  /**
   * @brief rgbCurve constructor
   * @param[in] path - filepath of an rgbCurve file
   */
  explicit rgbCurve(const std::string &path);

  void resize(std::size_t newSize)
  {
    for(auto &curve : _data)
      curve.resize(newSize);
    assert(getSize() == newSize);
  }

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

  void setFunction(EFunctionType functionType);

  /**
    * @brief Set curves to linear
    */
  void setLinear();

  /**
    * @brief Set curves to gamma
    */
  void setGamma();

  /**
    * @brief Set curves to Grossberg mean function of emor model
    */
  void setEmorInv(size_t dim = 0);


  /**
    * @brief Set curves to Grossberg mean function of emor model
    */
  void setEmor(size_t dim = 0);

  /**
    *@brief Set curves to gaussian
    */
  void setGaussian(double mu = 0.5, double sigma = 1.0 / (5.0 * sqrt(2.0)));


  /**
    *@brief Set curves to triangular
    */
  void setTriangular();

  /**
    * @brief Set curves to plateau
    */
  void setPlateau(float weight = 8.0f);

  void setPlateauSigmoid(float cA = 0.2, float wA = 0.5, float cB = 0.85, float wB = 0.22);

  /**
    *@brief Set curves to log10
    */
  void setLog10();

  /**
    * @brief inverse all values of the image
    * rgbCurve(i) = 1/i
    */
  void inverseAllValues();

  void freezeFirstPartValues();

  void freezeSecondPartValues();

  void invertAndScaleSecondPart(float scale);

  /**
    * @brief change all value of the image by their absolute value
    */
  void setAllAbsolute();

  void applyGamma(float gamma = 2.2);

  void applyGammaInv(float gamma = 2.2);

  /**
    * @brief normalize the curve
    */
  void normalize();

  /**
    * @brief scale the curve between 0 and 1
    */
  void scale();

  /**
    * @brief scale the curve between 0 and 1
    */
  void scaleChannelWise();

  /**
    * @brief interpolates all values at zero with the previous an the next value
    */
  void interpolateMissingValues();

  /**
    * @brief calculate the exponential of the curve
    */
  void exponential();


  /**
    * @brief Right accessor
    * @param[in] sample
    * @param[in] channel
    * @return the value at the index corresponding to the sample of the channel curve
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
    * @brief Operator*= Call multiply method with float parameter
    * @param[in] coefficient
    * @return rgbCurve of the multiplication
    */
  const rgbCurve operator*(const float coefficient);

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
    * @brief Multiply all value of an rgbCurve by a float coefficient
    * @param[in] coefficient
    * @return rgbCurve of the multiplication
    */
  const rgbCurve multiply(const float coefficient);

  /**
    * @brief Calculate the mean curve of all curves of an rgbCurve
    * @return rgbCurve of the mean curve
    */
  const rgbCurve meanCurves() const;

  /**
    * @brief Write in a csv file
    * @param[in] path
    */
  void write(const std::string &path, const std::string &name = "rgbCurve") const;

  /**
  * @brief Write in an html file
  * @param[in] path
  */
  void writeHtml(const std::string &path, const std::string& title) const;

  /**
    * @brief Read and fill curves from a csv file
    * @param[in] path
    */
  void read(const std::string &path);

  bool isEmpty() const
  {
    return _data.front().empty();
  }

  /**
    * @brief Access curve value at the specified index and channel
    * @param[in] index
    * @param[in] channel
    * @return the value at the index of the channel curve
    */
  float& getValue(std::size_t index, std::size_t channel)
  {
    assert(channel < _data.size());
    assert(index < getSize());
    return _data.at(channel).at(index);
  }

  /**
    * @brief Access curve value at the specified index and channel
    * @param[in] index
    * @param[in] channel
    * @return the value at the index of the channel curve
    */
  float getValue(std::size_t index, std::size_t channel) const
  {
    assert(channel < _data.size());
    assert(index < getSize());
    return _data.at(channel).at(index);
  }

  /**
    * @brief Set curve value at the specified index and channel
    * @param[in] index
    * @param[in] channel
    * @param[in] new desired value
    */
  void setValue(std::size_t index, std::size_t channel, float value)
  {
    assert(channel < _data.size());
    assert(index < getSize());
    _data.at(channel).at(index) = value;
  }

  /**
    * @brief Get inferior index value corresponding to float sample and fractionalPart between inferior and superior indexes
    * @param[in] sample between 0 and 1
    * @param[out] fractionalPart between inferior and superior indexes
    * @return inferior index of curve
    */
  std::size_t getIndex(float sample, float& fractionalPart) const
  {
    assert(getSize() != 0);

    float infIndex;
    float size = getSize() - 1.0f;
    float valueScaled = std::max(0.f, std::min(1.f, sample)) * size;
    fractionalPart = std::modf(valueScaled, &infIndex);

    return std::size_t(infIndex);
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
  /// Array containing the curves as vectors of float for each channel
  std::array< std::vector<float>, 3 > _data;
};

} // namespace hdr
} // namespace aliceVision
