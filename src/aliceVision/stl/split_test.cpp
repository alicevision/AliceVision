// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "split.hpp"

#include <string>
#include <vector>

#define BOOST_TEST_MODULE stlSplit
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_CASE(Split_stringEmpty)
{
  std::string sInput = "";
  std::string sDelimiter = " ";
  std::vector<std::string> vec_str;
  BOOST_CHECK(! stl::split(sInput, sDelimiter, vec_str));
  BOOST_CHECK_EQUAL( 1, vec_str.size() );
}

BOOST_AUTO_TEST_CASE(Split_delimiterEmpty)
{
  std::string sInput = "A string";
  std::string sDelimiter = "";
  std::vector<std::string> vec_str;
  BOOST_CHECK(! stl::split(sInput, sDelimiter, vec_str));
  BOOST_CHECK_EQUAL( 0, vec_str.size() );
}


BOOST_AUTO_TEST_CASE(Split_delimiterNotExist)
{
  std::string sInput = "A string";
  std::string sDelimiter = "_";
  std::vector<std::string> vec_str;
  BOOST_CHECK(! stl::split(sInput, sDelimiter, vec_str));
  BOOST_CHECK_EQUAL( 1, vec_str.size() );
}

BOOST_AUTO_TEST_CASE(Split_delimiterExist)
{
  std::string sInput = "A string";
  std::string sDelimiter = " ";
  std::vector<std::string> vec_str;
  BOOST_CHECK( stl::split(sInput, sDelimiter, vec_str));
  BOOST_CHECK_EQUAL( 2, vec_str.size() );
}

BOOST_AUTO_TEST_CASE(Split_stringSplit3part)
{
  std::string sInput = "A string useless";
  std::string sDelimiter = " ";
  std::vector<std::string> vec_str;
  BOOST_CHECK( stl::split(sInput, sDelimiter, vec_str));
  BOOST_CHECK_EQUAL( 3, vec_str.size() );
}

BOOST_AUTO_TEST_CASE(Split_ontheSameString)
{
	std::string sInput = "";
	std::string sDelimiter = ";";
	std::vector<std::string> vec_str;
	vec_str.push_back("foo;");
  BOOST_CHECK( stl::split(vec_str[0], sDelimiter, vec_str));
	BOOST_CHECK_EQUAL( 2, vec_str.size() );
}
