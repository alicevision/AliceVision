// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/metric.hpp>

#include <iostream>
#include <string>

#define BOOST_TEST_MODULE matchingMetric

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision;
using namespace feature;

template<typename Metric>
typename Metric::ResultType DistanceT()
{
  typename Metric::ElementType array1[] = {0, 1, 2, 3, 4, 5, 6, 7};
  typename Metric::ElementType array2[] = {7, 6, 5, 4, 3, 2, 1, 0};
  Metric metric;
  return metric(array1, array2, 8);
}

BOOST_AUTO_TEST_CASE(Metric_L2_Simple)
{
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Simple<unsigned char> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Simple<short> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Simple<int> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Simple<float> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Simple<double> >());
}

BOOST_AUTO_TEST_CASE(Metric_L2_Vectorized)
{
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Vectorized<unsigned char> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Vectorized<short> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Vectorized<int> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Vectorized<float> >());
  BOOST_CHECK_EQUAL(168, DistanceT<L2_Vectorized<double> >());
}

BOOST_AUTO_TEST_CASE(Metric_HAMMING_BITSET)
{
  std::bitset<8> a(std::string("01010101"));
  std::bitset<8> b(std::string("10101010"));
  std::bitset<8> c(std::string("11010100"));

  HammingBitSet<std::bitset<8> > metricHamming;
  BOOST_CHECK_EQUAL(8, metricHamming(&a,&b,1));
  BOOST_CHECK_EQUAL(0, metricHamming(&a,&a,1));
  BOOST_CHECK_EQUAL(2, metricHamming(&a,&c,1));
  
  Hamming< unsigned char > metricHammingUchar;
  
  BOOST_CHECK_EQUAL(8, metricHammingUchar(reinterpret_cast<unsigned char *>(&a),reinterpret_cast<unsigned char *>(&b),1));
  BOOST_CHECK_EQUAL(0, metricHammingUchar(reinterpret_cast<unsigned char *>(&a),reinterpret_cast<unsigned char *>(&a),1));
  BOOST_CHECK_EQUAL(2, metricHammingUchar(reinterpret_cast<unsigned char *>(&a),reinterpret_cast<unsigned char *>(&c),1));
}

BOOST_AUTO_TEST_CASE(Metric_HAMMING_BITSET_RAW_MEMORY_64BITS)
{
  const int COUNT = 4;
  std::bitset<64> tab[COUNT];
  // Zeros
  for(int i = 0; i < 64; ++i) {  tab[0][i] = 0;  }
  // 0101 ...
  for(int i = 0; i < 64; ++i) {  tab[1][i] = i%2 == 0;  }
  // 00110011...
  for(int i = 0; i < 64; ++i) {  tab[2][i] = (i/2)%2 == 0;  }
  // 000111000111...
  for(int i = 0; i < 64; ++i) {  tab[3][i] = (i/3)%2 == 0;  }

  // ground truth hamming distances between bit array
  const double gtDist[] =
  {0, 32, 32, 33, 32,
   0, 32, 21, 32, 32,
   0, 31, 33, 21, 31, 0};

  HammingBitSet<std::bitset<8> > metricHammingBitSet;
  Hamming< unsigned char > metricHamming;
  size_t cpt = 0;
  for (size_t i = 0; i < COUNT; ++i)
  {
    for (size_t j = 0; j < COUNT; ++j)
    {
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHammingBitSet(&tab[i],&tab[j], 1));
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHamming((uint64_t*)&tab[i],(uint64_t*)&tab[j], sizeof(uint64_t)));
      //Check distance symmetry
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHammingBitSet(&tab[j],&tab[i], 1));
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHamming((uint64_t*)&tab[j],(uint64_t*)&tab[i], sizeof(uint64_t)));
      ++cpt;
    }
  }
}

BOOST_AUTO_TEST_CASE(Metric_HAMMING_BITSET_RAW_MEMORY_32BITS)
{
  const int COUNT = 4;
  std::bitset<32> tab[COUNT];
  // Zeros
  for(int i = 0; i < 32; ++i) {  tab[0][i] = 0;  }
  // 0101 ...
  for(int i = 0; i < 32; ++i) {  tab[1][i] = i%2 == 0;  }
  // 00110011...
  for(int i = 0; i < 32; ++i) {  tab[2][i] = (i/2)%2 == 0;  }
  // 000111000111...
  for(int i = 0; i < 32; ++i) {  tab[3][i] = (i/3)%2 == 0;  }

  // ground truth hamming distances between bit array
  const double gtDist[] =
  {0, 16, 16, 17, 16,
  0, 16, 11, 16, 16,
  0, 17, 17, 11, 17, 0};

  HammingBitSet<std::bitset<8> > metricHammingBitSet;
  Hamming< unsigned char > metricHamming;
  size_t cpt = 0;
  for (size_t i = 0; i < COUNT; ++i)
  {
    for (size_t j = 0; j < COUNT; ++j)
    {
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHammingBitSet(&tab[i],&tab[j], 1));
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHamming((uint32_t*)&tab[i],(uint32_t*)&tab[j], sizeof(uint32_t)));
      //Check distance symmetry
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHammingBitSet(&tab[j],&tab[i], 1));
      BOOST_CHECK_EQUAL(gtDist[cpt], metricHamming((uint32_t*)&tab[j],(uint32_t*)&tab[i], sizeof(uint32_t)));
      ++cpt;
    }
  }
}
