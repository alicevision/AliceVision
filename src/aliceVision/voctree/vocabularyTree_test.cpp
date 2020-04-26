// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/voctree/Database.hpp>

#include <iostream>
#include <fstream>
#include <vector>

#define BOOST_TEST_MODULE vocabularyTree

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision::voctree;

BOOST_AUTO_TEST_CASE(database)
{
  const int cardDocuments = 10;
  const int cardWords = 12;

  // Create a documents vector
  vector<vector<Word>> documentsToInsert;
  documentsToInsert.resize(cardDocuments);
  for(int i = 0; i < documentsToInsert.size(); ++i)
  {
    documentsToInsert[i].resize(cardWords);
    for(int j = 0; j < cardWords; ++j)
    {
      documentsToInsert[i][j] = cardWords * i + j;
    }
  }

  // Create the databases
  Database db(documentsToInsert.size() * documentsToInsert[0].size());
  for(int i = 0; i < documentsToInsert.size(); ++i)
  {
    SparseHistogram histo;
    computeSparseHistogram(documentsToInsert[i], histo);
    db.insert(i, histo);
  }

  // Compute weights
  db.computeTfIdfWeights();

  // Check returned matches for a given document
  for(int i = 0; i < documentsToInsert.size(); i++)
  {
    // Create match vectors
    vector<DocMatch> match(1);
    // Query both databases with the same document
    db.find(documentsToInsert[i], 1, match, "classic");
    // Check the matches scores are 0 (or near)
    BOOST_CHECK_SMALL(static_cast<double>(match[0].score), 0.001);
  }
}
