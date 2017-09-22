// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <vector>
#include <string>

namespace stl
{

/**
 * Split an input string with a delimiter and fill a string vector
 */
inline bool split ( const std::string src, const std::string& delim, std::vector<std::string>& vec_value )
{
  bool bDelimiterExist = false;
  if ( !delim.empty() )
  {
    vec_value.clear();
    std::string::size_type start = 0;
    std::string::size_type end = std::string::npos -1;
    while ( end != std::string::npos )
    {
      end = src.find ( delim, start );
      vec_value.push_back ( src.substr ( start, end - start ) );
      start = end + delim.size();
    }
    if ( vec_value.size() >= 2 )
      bDelimiterExist = true;
  }
  return bDelimiterExist;
}

} // namespace stl
