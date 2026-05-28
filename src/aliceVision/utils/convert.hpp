// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <iomanip>
#include <map>
#include <sstream>
#include <string>

namespace aliceVision {
namespace utils {

inline std::string toStringZeroPadded(std::size_t i, std::size_t zeroPadding)
{
    std::stringstream ss;
    ss << std::setw(zeroPadding) << std::setfill('0') << i;
    return ss.str();
}

/**
 * @brief Converts a flat vector of strings representing key-value pairs
 *        into a std::map of string-to-string.
 *
 * @param dict A vector of strings where each pair of elements (i, i+1)
 *             represents a key and value, possibly with formatting characters.
 * @return std::map<std::string, std::string> A map containing cleaned key-value pairs.
 *
 * @code
 * std::vector<std::string> dict = {"{key1:", "value1,", "key2:", "value2}"};
 * auto result = dictStringToStringMap(dict);
 * // result => { {"key1", "value1"}, {"key2", "value2"} }
 * @endcode
 */
inline std::map<std::string, std::string> dictStringToStringMap(std::vector<std::string> dict)
{
    std::map<std::string, std::string> stringMap;

    for (std::size_t i = 0; i < dict.size(); i += 2)
    {
        std::string keyString = std::string(dict[i]);
        std::string valueString = std::string(dict[i + 1]);

        if (keyString[0] == '{')
        {
            keyString = keyString.substr(1, keyString.size() - 1);
        }
        if (keyString[keyString.size() - 1] == ':')
        {
            keyString = keyString.substr(0, keyString.size() - 1);
        }

        if (valueString[valueString.size() - 1] == ',' || valueString[valueString.size() - 1] == '}')
        {
            valueString = valueString.substr(0, valueString.size() - 1);
        }

        // Ensure both strings are not empty after being parsed.
        // If one of them is, do not add the pair to the map.
        if (keyString.size() > 0 && valueString.size() > 0)
        {
            stringMap.insert(std::pair<std::string, std::string>(keyString, valueString));
        }
    }

    return stringMap;
}

}  // namespace utils
}  // namespace aliceVision
