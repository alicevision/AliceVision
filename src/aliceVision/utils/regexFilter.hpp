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

 /**
 * @brief Create regex from a string filter (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more)
 * @param[in] str - Input string filter
 * @return the resulting regex
 */
std::regex filterToRegex(std::string str)
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
 * @brief Template function that allows you to filter any type of stl like string container by deleting in it the values that do not match the filter
 * @param[in] strContainer - Any container that contains strings
 * @param[in] filter - String represent the filter to be applied
 */
template <typename C, typename T = std::decay_t<decltype(*std::declval<C>().begin())>, typename = std::enable_if_t<std::is_convertible_v<T, std::string>>>
void filterStringCollection(C& strContainer, const std::string& filter)
{
    const std::regex regex = filterToRegex(filter);
    const auto begin = strContainer.begin();
    const auto end = strContainer.end();
    strContainer.erase(std::remove_if(begin, end, [&regex](const std::string& str) { return !std::regex_match(str, regex); }), end);
}