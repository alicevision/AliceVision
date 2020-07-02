// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string/replace.hpp>

#include <regex>
#include <string>

namespace aliceVision {
namespace utils
{
 /**
 * @brief Create regex from a string filter (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more)
 * @param[in] str - Input string filter
 * @return the resulting regex
 */
inline std::regex filterToRegex(std::string str)
{
    boost::replace_all(str, ".", "\\."); // escape "."
    boost::replace_all(str, "@", "[0-9]+"); // one @ correspond to one or more digits
    boost::replace_all(str, "#", "[0-9]"); // each # in pattern correspond to a digit
    boost::replace_all(str, "*", "(.*)");
    boost::replace_all(str, "?", "(.)");
    std::regex regexFilter;
    try
    {
        regexFilter = std::regex(str);
    }
    catch(const std::regex_error& e)
    {
        ALICEVISION_LOG_ERROR("[filterToRegex] regex_error caught: " << std::string(e.what()));
        throw std::invalid_argument("Invalid regex conversion, your input filter may be invalid.\nGenerated regex: '" + str + "'");
    }
    return regexFilter;
}

/**
 * @brief Function that allows you to filter vector of strings by deleting in it the values that do not match the filter
 * @param[in] <td::vector<std::string> - Vector that contains strings
 * @param[in] filter - String represent the filter to be applied
 */
inline void filterStrings(std::vector<std::string>& strVec, const std::string& filter)
{
    const std::regex regex = filterToRegex(filter);
    const auto begin = strVec.begin();
    const auto end = strVec.end();
    strVec.erase(std::remove_if(begin, end, [&regex](const std::string& str) { return !std::regex_match(str, regex); }), end);
}

} // namespace utils
} // namespace aliceVision