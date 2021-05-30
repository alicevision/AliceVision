// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "rgbCurve.hpp"
#include <functional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <limits>

#include <aliceVision/system/Logger.hpp>

#include <dependencies/htmlDoc/htmlDoc.hpp>


namespace aliceVision {
namespace hdr {
    
rgbCurve::rgbCurve(std::size_t size)
{
    for(auto &curve : _data)
    {
        curve.resize(size);
    }
    setZero();
}

rgbCurve::rgbCurve(const std::string &path)
{
      read(path);
}

void rgbCurve::setFunction(EFunctionType functionType)
{
    switch(functionType)
    {
        case EFunctionType::LINEAR:     setLinear(); return;
        case EFunctionType::GAUSSIAN:   setGaussian(); return;
        case EFunctionType::TRIANGLE:   setTriangular(); return;
        case EFunctionType::PLATEAU:    setPlateauSigmoid(); return;
        case EFunctionType::GAMMA:      setGamma(); return;
        case EFunctionType::LOG10:      setLog10(); return;
    }
    throw std::out_of_range("Invalid function type enum");
}

void rgbCurve::setLinear() 
{
    const float coefficient = 1.f / static_cast<float>(getSize() - 1);
    for(std::size_t i = 0; i < getSize(); ++i)
    {
        setAllChannels(i, i * coefficient);
    }
}

void rgbCurve::setGamma()
{
    const float coefficient = 1.f / static_cast<float>(getSize() - 1);
    for(std::size_t i = 0; i < getSize(); ++i)
    {
        setAllChannels(i, std::pow(4.0f * i * coefficient, 1.7f) + 1e-4);
    }
}

void rgbCurve::setEmor(size_t dim )
{
  const std::size_t emorSize = std::pow(2, 10);
  const std::size_t curveSize = getSize();
  const double* ptrf0 = getEmorCurve(dim);
  
  std::vector<double> f0;
  if(curveSize == emorSize)
  {
    for(auto &curve : _data)
      curve.assign(ptrf0, ptrf0 + emorSize);
  }
  else if(emorSize > curveSize)
  {
    f0.assign(ptrf0, ptrf0 + emorSize);
    std::vector<float> emor = std::vector<float>(f0.begin(), f0.end());

    std::size_t step = emorSize/curveSize;
    for(auto &curve : _data)
    {
      for(std::size_t i = 0; i<curveSize; ++i)
        curve.at(i) = emor.at(step*i);
    }
  }
  else
  {
    f0.assign(ptrf0, ptrf0 + emorSize);
    std::vector<float> emor = std::vector<float>(f0.begin(), f0.end());

    std::size_t step = curveSize/emorSize;
    for(auto &curve : _data)
    {
      for(std::size_t i = 0; i<emorSize-1; ++i)
        curve.at(i*step) = emor.at(i);
      curve.at(emorSize*step-1) = emor.at(emorSize-1);
    }
    interpolateMissingValues();
  }
}

void rgbCurve::setEmorInv(size_t dim )
{
  const std::size_t emorSize = std::pow(2, 10);
  const std::size_t curveSize = getSize();
  const double* ptrf0 = getEmorInvCurve(dim);
  
  std::vector<double> f0;
  if(curveSize == emorSize)
  {
    for(auto &curve : _data)
      curve.assign(ptrf0, ptrf0 + emorSize);
  }
  else if(emorSize > curveSize)
  {
    f0.assign(ptrf0, ptrf0 + emorSize);
    std::vector<float> emor = std::vector<float>(f0.begin(), f0.end());

    std::size_t step = emorSize/curveSize;
    for(auto &curve : _data)
    {
      for(std::size_t i = 0; i<curveSize; ++i)
        curve.at(i) = emor.at(step*i);
    }
  }
  else
  {
    f0.assign(ptrf0, ptrf0 + emorSize);
    std::vector<float> emor = std::vector<float>(f0.begin(), f0.end());

    std::size_t step = curveSize/emorSize;
    for(auto &curve : _data)
    {
      for(std::size_t i = 0; i<emorSize-1; ++i)
        curve.at(i*step) = emor.at(i);
      curve.at(emorSize*step-1) = emor.at(emorSize-1);
    }
    interpolateMissingValues();
  }
}

void rgbCurve::setGaussian(double mu, double sigma)
{
    // https://www.desmos.com/calculator/s3q3ow1mpy
    for(std::size_t i = 0; i < getSize(); ++i)
    {
        float factor = i / (static_cast<float>(getSize() - 1)) - mu;
        setAllChannels(i, std::exp( -factor * factor / (2.0 * sigma * sigma)));
    }
}

void rgbCurve::setTriangular()
{
    const float coefficient = 2.f / static_cast<float>(getSize() - 1);
    for(std::size_t i = 0; i < getSize(); ++i)
    {
        float value = i * coefficient;
        if (value > 1.0f)
        {
            value = 2.0f - value;
        }
        setAllChannels(i, value);
    }
}


void rgbCurve::setPlateau(float weight)
{
    // https://www.desmos.com/calculator/mouwyuvjvw

    const float coefficient = 1.f / static_cast<float>(getSize() - 1);
    for(std::size_t i = 0; i < getSize(); ++i)
    {
        setAllChannels(i, 1.0f - std::pow((2.0f * i * coefficient - 1.0f), weight));
    }
}

inline float plateauSigmoidFunction(float cA, float wA, float cB, float wB, float x)
{
    // https://www.desmos.com/calculator/aoojidncmi
    return 1.0 / (1.0 + std::exp(10.0 * (x - cB) / wB)) - 1.0 / (1.0 + std::exp(10.0 * (x - cA) / wA));
}

void rgbCurve::setPlateauSigmoid(float cA, float wA, float cB, float wB)
{
    const float coefficient = 1.f / static_cast<float>(getSize() - 1);
    for (std::size_t i = 0; i < getSize(); ++i)
    {
        setAllChannels(i, plateauSigmoidFunction(cA, wA, cB, wB, i * coefficient));
    }
}

void rgbCurve::setLog10()
{
    const float coefficient = 1.f / static_cast<float>(getSize() - 1);
    const float logInverseNorm = 1.0f / 0.0625f;
    const float logInverseMaxValue = 1.0f / 1e8;
    for(std::size_t i = 0; i < getSize(); ++i)
    {
        setAllChannels(i, logInverseMaxValue * std::pow(10.0f, (i * coefficient * logInverseNorm) - 8.f));
    }
}

void rgbCurve::inverseAllValues()
{
    for(auto &curve : _data)
    {
        for(auto &value : curve)
        {
            if(value != 0.f)
            {
                value = 1.f / value;
            }
        }
    }
}

void rgbCurve::freezeFirstPartValues()
{
    for (auto &curve : _data)
    {
        std::size_t midIndex = (curve.size() / 2);
        for (std::size_t i = 0; i < midIndex; ++i)
        {
            curve[i] = curve[midIndex];
        }
    }
}

void rgbCurve::freezeSecondPartValues()
{
    for (auto &curve : _data)
    {
        std::size_t midIndex = (curve.size() / 2);
        for (std::size_t i = midIndex + 1; i < curve.size(); ++i)
        {
            curve[i] = curve[midIndex];
        }
    }
}

void rgbCurve::invertAndScaleSecondPart(float scale)
{
    for (auto &curve : _data)
    {
        for (std::size_t i = curve.size()/2; i < curve.size(); ++i)
        {
            curve[i] = (1.f - curve[i]) * scale;
        }
    }
}

void rgbCurve::setAllAbsolute()
{
    for(auto &curve : _data)
    {
        for(auto &value : curve)
        {
            value = std::abs(value);
        }
    }
}

inline float gammaFunction(float value, float gamma)
{
    // 1/0.45 = 2.22
    if (value < 0.018)
    {
        return 4.5 * value;
    }
    else
    {
        return 1.099 * std::pow(value, 0.45) - 0.099;
    }
}

inline float inverseGammaFunction(float value, float gamma)
{
    if (value <= 0.0812f)
    {
        return value / 4.5f;
    }
    else
    {
        return pow((value + 0.099f) / 1.099f, gamma);
    }
}

void rgbCurve::applyGamma(float gamma)
{
    for (auto &curve : _data)
    {
        for (auto &value : curve)
        {
            value = gammaFunction(value, gamma);
        }
    }
}

void rgbCurve::applyGammaInv(float gamma)
{
    for (auto &curve : _data)
    {
        for (auto &value : curve)
        {
            value = inverseGammaFunction(value, gamma);
        }
    }
}

void rgbCurve::normalize()
{
    for(auto &curve : _data)
    {
        std::size_t first = 0;
        std::size_t last = curve.size() - 1;

        // find first and last value not null
        for (; (first < curve.size()) && (curve[first] == 0) ; ++first);
        for (; (last > 0) && (curve[last] == 0)  ; --last);

        std::size_t middle = first + ((last - first) / 2);
        float midValue = curve[middle];

        if (midValue == 0.0f)
        {
            // find first non-zero middle response
            for (; (middle < last) && (curve[middle] == 0.0f) ; ++middle);
            midValue = curve[middle];
        }

//        ALICEVISION_LOG_TRACE("-> middle [" << middle << "]: " << midValue);
        const float coefficient = 1 / midValue;

        for(auto &value : curve)
        {
            value *= coefficient;
        }
    }
}

void rgbCurve::scale()
{
    float minTot = std::numeric_limits<float>::max();
    float maxTot = std::numeric_limits<float>::min();
    for(auto &curve : _data)
    {
        float minV = *std::min_element(curve.begin(), curve.end());
        float maxV = *std::max_element(curve.begin(), curve.end());
        minTot = std::min(minTot, minV);
        maxTot = std::max(maxTot, maxV);
    }
    for(auto &curve : _data)
        for(auto &value : curve)
            value = (value - minTot) / (maxTot - minTot);
}

void rgbCurve::scaleChannelWise()
{
    for(auto &curve : _data)
    {
        float minV = *std::min_element(curve.begin(), curve.end());
        float maxV = *std::max_element(curve.begin(), curve.end());
        for(auto &value : curve) {
            value = (value - minV) / (maxV - minV);
        }
    }       
}

void rgbCurve::interpolateMissingValues()
{
    for(auto &curve : _data)
    {
        std::size_t previousValidIndex = 0;
        for(std::size_t index = 1; index < curve.size(); ++index)
        {
            if(curve[index] != 0.0f)
            {
                if(previousValidIndex+1 < index)
                {
                    const float inter = (curve[index] - curve[previousValidIndex]) / (index - previousValidIndex);
                    for(std::size_t j = previousValidIndex+1; j < index; ++j)
                    {
                        curve[j] = curve[previousValidIndex] + inter * (j-previousValidIndex);
                    }
                }
                previousValidIndex = index;
            }
        }
    }
}

void rgbCurve::exponential()
{
    for(auto &curve : _data)
    {
        for(auto &value : curve)
            value = std::exp(value);
    }
}

float rgbCurve::operator() (float sample, std::size_t channel) const
{
  assert(channel < _data.size());
  
  float fractionalPart = 0.0;
  std::size_t infIndex = getIndex(sample, fractionalPart);
  
  /* Do not interpolate 1.0 */
  if (infIndex == getSize() - 1) {
    return _data[channel][infIndex];
  }

  return (1.0f - fractionalPart) * _data[channel][infIndex] + fractionalPart * _data[channel][infIndex + 1];
}

const rgbCurve rgbCurve::operator+(const rgbCurve &other) const
{
  return sum(other);
}

const rgbCurve rgbCurve::operator-(const rgbCurve &other) const
{
  return subtract(other);
}

void rgbCurve::operator*=(const rgbCurve &other)
{
  multiply(other);
}

const rgbCurve rgbCurve::operator*(const float coefficient)
{
    return multiply(coefficient);
}

const rgbCurve rgbCurve::sum(const rgbCurve &other) const
{
    assert(getSize() == other.getSize());

    rgbCurve sum = rgbCurve(*this);
    for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
    {
        auto &sumCurve = sum.getCurve(channel);
        const auto &otherCurve = other.getCurve(channel);

        //sum member channel by the other channel
        std::transform (sumCurve.begin(), sumCurve.end(), otherCurve.begin(), sumCurve.begin(), std::plus<float>());
    }
    return sum;
}

const rgbCurve rgbCurve::subtract(const rgbCurve &other) const
{
    assert(getSize() == other.getSize());

    rgbCurve sub = rgbCurve(*this);
    for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
    {
        auto &subCurve = sub.getCurve(channel);
        const auto &otherCurve = other.getCurve(channel);

        //subtract member channel by the other channel
        std::transform (subCurve.begin(), subCurve.end(), otherCurve.begin(), subCurve.begin(), std::minus<float>());
    }
    return sub;
}

void rgbCurve::multiply(const rgbCurve &other)
{
    assert(getSize() == other.getSize());

    for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
    {
        auto &curve = getCurve(channel);
        const auto &otherCurve = other.getCurve(channel);

        //multiply member channel by the other channel
        std::transform (curve.begin(), curve.end(), otherCurve.begin(), curve.begin(), std::multiplies<float>());
    }
}

const rgbCurve rgbCurve::multiply(const float coefficient)
{
    for(auto &curve : _data)
        for(auto &value : curve)    value *= coefficient;
    return (*this);
}

const rgbCurve rgbCurve::meanCurves() const
{
  std::size_t nbChannels = getNbChannels();
  rgbCurve mean = rgbCurve(nbChannels);
  std::vector<float> calculateMean(getSize());

  for(auto &curve : _data)
    std::transform (calculateMean.begin(), calculateMean.end(), curve.begin(), calculateMean.begin(), std::plus<float>());

  for(std::size_t channel = 0; channel<nbChannels; ++channel)
    mean.getCurve(channel) = calculateMean;

  return mean.multiply(float(1.f/nbChannels));
}

void rgbCurve::write(const std::string &path, const std::string &name) const
{
    std::ofstream file(path);

    if(!file)
    {
        throw std::logic_error("Can't create curves file");
    }

    std::string text(name + ",Red,Green,Blue,\n");
    for(std::size_t index = 0; index < getSize(); ++index)
    {
        text += std::to_string(index) + ",";
        for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
        {
            text += std::to_string(_data[channel][index]) + ",";
        }
        text += "\n";
    }
    file << text;
    file.close();
}

void rgbCurve::writeHtml(const std::string& path, const std::string& title) const
{
    using namespace htmlDocument;

    std::vector<double> xBin(getCurveRed().size());
    std::iota(xBin.begin(), xBin.end(), 0);

    std::pair< std::pair<double, double>, std::pair<double, double> > range = autoJSXGraphViewport<double>(xBin, getCurveRed());

    JSXGraphWrapper jsxGraph;
    jsxGraph.init(title, 800, 600);

    jsxGraph.addXYChart(xBin, getCurveRed(), "line", "ff0000");
    jsxGraph.addXYChart(xBin, getCurveGreen(), "line", "00ff00");
    jsxGraph.addXYChart(xBin, getCurveBlue(), "line", "0000ff");

    jsxGraph.UnsuspendUpdate();
    jsxGraph.setViewport(range);
    jsxGraph.close();

    // save the reconstruction Log
    std::ofstream htmlFileStream(path.c_str());
    htmlFileStream << htmlDocumentStream(title).getDoc();
    htmlFileStream << jsxGraph.toStr();
}


void rgbCurve::read(const std::string &path)
{
    std::ifstream file(path);
    std::vector <std::vector <std::string> > fileData;

    if(!file)
    {
        throw std::logic_error("Can't open curves file");
    }

    //create fileData
    while (file)
    {
        std::string line;
        if (!getline( file, line )) break;

        std::istringstream sline( line );
        std::vector<std::string> record;
        while (sline)
        {
            std::string value;
            if (!getline( sline, value, ',' )) break;
            record.push_back( value );
        }
        fileData.push_back( record );
    }

    //clear rgbCurve
    for(auto &curve : _data)
    {
        curve.clear();
    }

    //fill rgbCurve

    try
    {
        for(std::size_t line = 1; line < fileData.size(); ++line)
        {
            _data[0].push_back(std::stof(fileData[line][1]));
            _data[1].push_back(std::stof(fileData[line][2]));
            _data[2].push_back(std::stof(fileData[line][3]));
        }
    }
    catch(std::exception &e)
    {
        throw std::logic_error("Invalid Curve File");
    }

    file.close();
}

double rgbCurve::sumAll(const rgbCurve &curve)
{
    double sum = 0.0f;
    for(std::size_t channel = 0; channel < curve.getNbChannels(); ++channel)
    {
        auto const &sumCurve = curve.getCurve(channel);

        for(auto value : sumCurve)
        {
            sum += value;
        }
    }
    return sum;
}

} // namespace hdr
} // namespace aliceVision
