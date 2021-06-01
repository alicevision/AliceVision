// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sensorDB/parseDatabase.hpp>

#include <boost/filesystem.hpp>

#include <string>

#define BOOST_TEST_MODULE parseDatabase

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision::sensorDB;
namespace fs = boost::filesystem;

static const std::string sDatabase = (fs::path(THIS_SOURCE_DIR) / "cameraSensors.db").string();

BOOST_AUTO_TEST_CASE(InvalidDatabase)
{
  std::vector<Datasheet> vec_database;
  const std::string sfileDatabase = std::string(THIS_SOURCE_DIR);

  BOOST_CHECK(! parseDatabase( sfileDatabase, vec_database ) );
  BOOST_CHECK( vec_database.empty() );
}

BOOST_AUTO_TEST_CASE(ValidDatabase)
{
  std::vector<Datasheet> vec_database;
  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK( !vec_database.empty() );
}

BOOST_AUTO_TEST_CASE(ParseDatabaseSD900)
{
  std::vector<Datasheet> vec_database;
  Datasheet datasheet;
  const std::string sModel = "Canon PowerShot SD900";
  const std::string sBrand = "Canon";

  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK( getInfo( sBrand, sModel, vec_database, datasheet ) );
  BOOST_CHECK_EQUAL( "Canon", datasheet._brand );
  BOOST_CHECK_EQUAL( "Canon PowerShot SD900", datasheet._model );
  BOOST_CHECK_EQUAL( 7.144, datasheet._sensorWidth );
}

BOOST_AUTO_TEST_CASE(ParseDatabaseA710_IS)
{
  std::vector<Datasheet> vec_database;
  Datasheet datasheet;
  const std::string sModel = "Canon PowerShot A710 IS";
  const std::string sBrand = "Canon";

  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK( getInfo( sBrand, sModel, vec_database, datasheet ) );
  BOOST_CHECK_EQUAL( "Canon", datasheet._brand );
  BOOST_CHECK_EQUAL( "Canon PowerShot A710 IS", datasheet._model );
  BOOST_CHECK_EQUAL( 5.744, datasheet._sensorWidth );
}

BOOST_AUTO_TEST_CASE(ParseDatabaseNotExist)
{
  std::vector<Datasheet> vec_database;
  Datasheet datasheet;
  const std::string sModel = "NotExistModel";
  const std::string sBrand = "NotExistBrand";

  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK(! getInfo( sBrand, sModel, vec_database, datasheet ) );
}


BOOST_AUTO_TEST_CASE(ParseDatabaseCanon_EOS_550D)
{
  std::vector<Datasheet> vec_database;
  Datasheet datasheet;
  const std::string sModel = "Canon EOS 550D";
  const std::string sBrand = "Canon";

  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK( getInfo( sBrand, sModel, vec_database, datasheet ) );
  BOOST_CHECK_EQUAL( 22.3, datasheet._sensorWidth );
}

BOOST_AUTO_TEST_CASE(ParseDatabaseCanon_EOS_5D_Mark_II)
{
  std::vector<Datasheet> vec_database;
  Datasheet datasheet;
  const std::string sModel = "Canon EOS 5D Mark II";
  const std::string sBrand = "Canon";

  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK( getInfo( sBrand, sModel, vec_database, datasheet ) );
  BOOST_CHECK_EQUAL( 36, datasheet._sensorWidth );
}

BOOST_AUTO_TEST_CASE(ParseDatabaseCanon_EOS_1100D)
{
  std::vector<Datasheet> vec_database;
  Datasheet datasheet;
  const std::string sModel = "Canon EOS 1100D";
  const std::string sBrand = "Canon";

  BOOST_CHECK( parseDatabase( sDatabase, vec_database ) );
  BOOST_CHECK( getInfo( sBrand, sModel, vec_database, datasheet ) );
  BOOST_CHECK_EQUAL( 22.2, datasheet._sensorWidth );
}
