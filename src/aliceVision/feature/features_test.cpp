// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/feature/feature.hpp"

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>

#define BOOST_TEST_MODULE Feature

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace std;
using std::string;
using namespace aliceVision;
using namespace aliceVision::feature;

// Define a feature and a container of features
typedef PointFeature Feature_T;
typedef std::vector<Feature_T> Feats_T;

// Define a descriptor and a container of descriptors
static const int DESC_LENGTH = 128;
typedef Descriptor<float, DESC_LENGTH> Desc_T;
typedef std::vector<Desc_T> Descs_T;

//--
//-- Features interface test
//--

static const int CARD = 12;

BOOST_AUTO_TEST_CASE(featureIO_NON_EXISTING_FILE) {

  // Try to read a non-existing feature file
  Feats_T vec_feats;
  BOOST_CHECK_THROW(loadFeatsFromFile("x.feat", vec_feats), std::exception);

  // Try to read a non-existing descriptor file
  Descs_T vec_descs;
  BOOST_CHECK_THROW(loadDescsFromFile("x.desc", vec_descs), std::exception);
  BOOST_CHECK_THROW(loadDescsFromBinFile("x.desc", vec_descs), std::exception);
}

BOOST_AUTO_TEST_CASE(featureIO_ASCII) {
  Feats_T vec_feats;
  for(int i = 0; i < CARD; ++i)  {
    vec_feats.push_back(Feature_T(i, i*2, i*3, i*4));
  }

  //Save them to a file
  BOOST_CHECK_NO_THROW(saveFeatsToFile("tempFeats.feat", vec_feats));

  //Read the saved data and compare to input (to check write/read IO)
  Feats_T vec_feats_read;
  BOOST_CHECK_NO_THROW(loadFeatsFromFile("tempFeats.feat", vec_feats_read));
  BOOST_CHECK_EQUAL(CARD, vec_feats_read.size());

  for(int i = 0; i < CARD; ++i) {
    BOOST_CHECK_EQUAL(vec_feats[i], vec_feats_read[i]);
    BOOST_CHECK_EQUAL(vec_feats[i].coords(), vec_feats_read[i].coords());
    BOOST_CHECK_EQUAL(vec_feats[i].scale(), vec_feats_read[i].scale());
    BOOST_CHECK_EQUAL(vec_feats[i].orientation(), vec_feats_read[i].orientation());
  }
}

//--
//-- Descriptors interface test
//--
BOOST_AUTO_TEST_CASE(descriptorIO_ASCII) {
  // Create an input series of descriptor
  Descs_T vec_descs;
  for(int i = 0; i < CARD; ++i)  {
    Desc_T desc;
    for (int j = 0; j < DESC_LENGTH; ++j)
      desc[j] = i*DESC_LENGTH+j;
    vec_descs.push_back(desc);
  }

  //Save them to a file
  BOOST_CHECK_NO_THROW(saveDescsToFile("tempDescs.desc", vec_descs));

  //Read the saved data and compare to input (to check write/read IO)
  Descs_T vec_descs_read;
  BOOST_CHECK_NO_THROW(loadDescsFromFile("tempDescs.desc", vec_descs_read));
  BOOST_CHECK_EQUAL(CARD, vec_descs_read.size());

  for(int i = 0; i < CARD; ++i) {
    for (int j = 0; j < DESC_LENGTH; ++j)
      BOOST_CHECK_EQUAL(vec_descs[i][j], vec_descs_read[i][j]);
  }
}

//Test binary export of descriptor
BOOST_AUTO_TEST_CASE(descriptorIO_BINARY) {
  // Create an input series of descriptor
  Descs_T vec_descs;
  for(int i = 0; i < CARD; ++i)
  {
    Desc_T desc;
    for (int j = 0; j < DESC_LENGTH; ++j)
      desc[j] = i*DESC_LENGTH+j;
    vec_descs.push_back(desc);
  }

  //Save them to a file
  BOOST_CHECK_NO_THROW(saveDescsToBinFile("tempDescsBin.desc", vec_descs));

  //Read the saved data and compare to input (to check write/read IO)
  Descs_T vec_descs_read;
  BOOST_CHECK_NO_THROW(loadDescsFromBinFile("tempDescsBin.desc", vec_descs_read));
  BOOST_CHECK_EQUAL(CARD, vec_descs_read.size());

  for(int i = 0; i < CARD; ++i) {
    for (int j = 0; j < DESC_LENGTH; ++j)
      BOOST_CHECK_EQUAL(vec_descs[i][j], vec_descs_read[i][j]);
  }
}
