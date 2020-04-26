// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/voctree/TreeBuilder.hpp>
#include <aliceVision/system/Logger.hpp>

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <vector>

#define BOOST_TEST_MODULE voctreeBuilder

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

BOOST_AUTO_TEST_CASE(voctreeBuilder)
{
  using namespace aliceVision;

  const std::string treeName = "test.tree";

  const std::size_t DIMENSION = 3;
  const std::size_t FEATURENUMBER = 100;

  const double kepsf = 10e-8;
  const std::size_t K = 4;
  const std::size_t LEVELS = 3;
  const std::size_t LEAVESNUMBER = std::pow(K, LEVELS);


  const std::size_t STEP = 1;

  typedef Eigen::Matrix<float, 1, DIMENSION> FeatureFloat;
  typedef std::vector<FeatureFloat, Eigen::aligned_allocator<FeatureFloat> > FeatureFloatVector;
  typedef std::vector<FeatureFloat* > FeaturePointerVector;

  // generate a random vector of features
  FeatureFloatVector features;
  features.reserve(FEATURENUMBER * LEAVESNUMBER);

  for(std::size_t i = 0; i < LEAVESNUMBER; ++i)
  {
    // at each i iteration translate the cluster by STEP*i
    for(std::size_t j = 0; j < FEATURENUMBER; ++j)
    {
      features.push_back((FeatureFloat::Random(1, DIMENSION) + Eigen::MatrixXf::Constant(1, DIMENSION, STEP * i) - Eigen::MatrixXf::Constant(1, DIMENSION, STEP * (LEAVESNUMBER - 1) / 2)) / ((STEP * (LEAVESNUMBER - 1) / 2) * sqrt(DIMENSION)));
      BOOST_CHECK(voctree::checkElements(features[j], "init"));
    }
  }

  // build the tree
  voctree::TreeBuilder<FeatureFloat> builder(FeatureFloat::Zero());
  builder.setVerbose(0);
  builder.kmeans().setRestarts(10);
  ALICEVISION_LOG_DEBUG("Building a tree of L = " << LEVELS << " levels with a branching factor of k = " << K);
  builder.build(features, K, LEVELS);
  ALICEVISION_LOG_DEBUG(builder.tree().centers().size() << " centers");

  // the centers should all be valid in this configuration
  std::vector<uint8_t> valid = builder.tree().validCenters();
  for(std::size_t i = 0; i < valid.size(); ++i)
    BOOST_CHECK(valid[i] != 0);

  builder.tree().save(treeName);

  voctree::MutableVocabularyTree<FeatureFloat> loadedtree;
  loadedtree.load(treeName);

  // check the centers are the same
  FeatureFloatVector centerOrig = builder.tree().centers();
  FeatureFloatVector centerLoad = loadedtree.centers();

  BOOST_CHECK_EQUAL(centerOrig.size(), centerLoad.size());

  voctree::L2<FeatureFloat, FeatureFloat> distance;
  for(std::size_t i = 0; i < centerOrig.size(); ++i)
  {
    BOOST_CHECK_SMALL(distance(centerOrig[i],centerLoad[i]), kepsf);
  }
//  voctree::printFeatVector( features ); 
}
