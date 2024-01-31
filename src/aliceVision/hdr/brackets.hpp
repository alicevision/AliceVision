// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace hdr {

enum class ECalibrationMethod
{
    LINEAR,
    DEBEVEC,
    GROSSBERG,
    LAGUERRE,
    AUTO,
};

/**
 * @brief Convert a calibration method from an ECalibrationMethod enum to its corresponding string
 * @param[in] calibrationMethod the ECalibrationMethod enum to convert to a string
 * @return the string containing the calibration method corresponding to the input ECalibration enum
 */
inline std::string ECalibrationMethod_enumToString(const ECalibrationMethod calibrationMethod)
{
    switch (calibrationMethod)
    {
        case ECalibrationMethod::AUTO:
            return "auto";
        case ECalibrationMethod::LINEAR:
            return "linear";
        case ECalibrationMethod::DEBEVEC:
            return "debevec";
        case ECalibrationMethod::GROSSBERG:
            return "grossberg";
        case ECalibrationMethod::LAGUERRE:
            return "laguerre";
    }
    throw std::out_of_range("Invalid method name enum");
}

/**
 * @brief Convert a string containing the calibration method name to its corresponding ECalibrationMethod enum
 * @param[in] calibrationMethodName the string containing the calibration method name to convert to an ECalibrationMethod enum
 * @return the ECalibrationMethod enum corresponding to the input calibration method
 */
inline ECalibrationMethod ECalibrationMethod_stringToEnum(const std::string& calibrationMethodName)
{
    std::string methodName = calibrationMethodName;
    std::transform(methodName.begin(), methodName.end(), methodName.begin(), ::tolower);

    if (methodName == "auto")
        return ECalibrationMethod::AUTO;
    if (methodName == "linear")
        return ECalibrationMethod::LINEAR;
    if (methodName == "debevec")
        return ECalibrationMethod::DEBEVEC;
    if (methodName == "grossberg")
        return ECalibrationMethod::GROSSBERG;
    if (methodName == "laguerre")
        return ECalibrationMethod::LAGUERRE;

    throw std::out_of_range("Invalid method name: '" + calibrationMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, ECalibrationMethod calibrationMethodName)
{
    os << ECalibrationMethod_enumToString(calibrationMethodName);
    return os;
}

inline std::istream& operator>>(std::istream& in, ECalibrationMethod& calibrationMethod)
{
    std::string token;
    in >> token;
    calibrationMethod = ECalibrationMethod_stringToEnum(token);
    return in;
}

/**
 * @brief Estimate the brackets information from the SfM data
 * @param[out] groups the estimated groups
 * @param[in] sfmData SfM data to estimate the brackets from
 * @param[in] countBrackets the number of brackets
 * @return false if an error occurs (e.g. an invalid SfMData file has been provided), true otherwise
 */
bool estimateBracketsFromSfmData(std::vector<std::vector<std::shared_ptr<aliceVision::sfmData::View>>>& groups,
                                 const aliceVision::sfmData::SfMData& sfmData,
                                 size_t countBrackets);

/**
 * @brief Select the target views (used for instance to define the expoure)
 * @param[out] targetViews the estimated target views
 * @param[in] groups the groups of Views corresponding to multi-bracketing. Warning: Needs to be sorted by exposure time.
 * @param[in] offsetRefBracketIndex 0 means center bracket and you can choose +N/-N to select the reference bracket
 * @param[in] targetIndexesFilename in case offsetRefBracketIndex is out of range, the number of views in a group target
 *                                  indexes can be read from a text file. If the file cannot be read or does not contain
 *                                  the expected number of values (same as view group number) and if
 *                                  offsetRefBracketIndex is out of range, the number of views then a clamped value of
 *                                  offsetRefBracketIndex is considered
 * @param[in] meanTargetedLuma mean targeted luma
 * @return the index of the target view
 */
int selectTargetViews(std::vector<std::shared_ptr<aliceVision::sfmData::View>>& targetViews,
                      std::vector<std::vector<std::shared_ptr<aliceVision::sfmData::View>>>& groups,
                      const int offsetRefBracketIndex,
                      const std::string& targetIndexesFilename,
                      const double meanTargetedLuma = 0.4);

}  // namespace hdr
}  // namespace aliceVision
