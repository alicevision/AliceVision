// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE imagePairListIO

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <fstream>

#include "ImagePairListIO.hpp"

using namespace aliceVision;
using namespace aliceVision::matchingImageCollection;

BOOST_AUTO_TEST_CASE(read_write_pairs_to_file)
{
    PairSet pairSetGT;
    pairSetGT.insert(std::make_pair(0, 1));
    pairSetGT.insert(std::make_pair(1, 2));
    pairSetGT.insert(std::make_pair(2, 0));

    PairSet pairSetGTsorted;
    pairSetGTsorted.insert(std::make_pair(0, 1));
    pairSetGTsorted.insert(std::make_pair(0, 2));
    pairSetGTsorted.insert(std::make_pair(1, 2));

    BOOST_CHECK(savePairsToFile("pairsT_IO.txt", pairSetGT));

    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO.txt", loaded_Pairs));
    BOOST_CHECK(std::equal(loaded_Pairs.begin(), loaded_Pairs.end(), pairSetGTsorted.begin()));
    std::remove("pairsT_IO.txt");
}

BOOST_AUTO_TEST_CASE(read_write_pairs_without_symmetry)
{
    // Test with useSymmetry=false to preserve order as (I, J) without sorting
    PairSet pairSetGT;
    pairSetGT.insert(std::make_pair(1, 0));
    pairSetGT.insert(std::make_pair(2, 1));
    pairSetGT.insert(std::make_pair(0, 2));

    BOOST_CHECK(savePairsToFile("pairsT_IO_nosym.txt", pairSetGT));

    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_nosym.txt", loaded_Pairs, false));
    
    // With useSymmetry=false, pairs should be loaded as-is
    BOOST_CHECK_EQUAL(loaded_Pairs.size(), 3);
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(0, 2)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(1, 0)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(2, 1)) != loaded_Pairs.end());
    
    std::remove("pairsT_IO_nosym.txt");
}

BOOST_AUTO_TEST_CASE(save_empty_pair_set)
{
    // Test saving an empty pair set
    PairSet emptySet;
    BOOST_CHECK(savePairsToFile("pairsT_IO_empty.txt", emptySet));
    
    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_empty.txt", loaded_Pairs));
    BOOST_CHECK(loaded_Pairs.empty());
    
    std::remove("pairsT_IO_empty.txt");
}

BOOST_AUTO_TEST_CASE(load_nonexistent_file)
{
    // Test loading from a file that doesn't exist
    PairSet pairs;
    BOOST_CHECK(!loadPairsFromFile("nonexistent_file_xyz123.txt", pairs));
    BOOST_CHECK(pairs.empty());
}

BOOST_AUTO_TEST_CASE(save_to_invalid_path)
{
    // Test saving to an invalid path
    PairSet pairSet;
    pairSet.insert(std::make_pair(0, 1));
    
    // Using a path that should fail on most systems
    BOOST_CHECK(!savePairsToFile("/invalid/path/that/does/not/exist/pairs.txt", pairSet));
}

BOOST_AUTO_TEST_CASE(read_single_pair)
{
    // Test reading a single pair
    PairSet pairSetGT;
    pairSetGT.insert(std::make_pair(5, 10));
    
    BOOST_CHECK(savePairsToFile("pairsT_IO_single.txt", pairSetGT));
    
    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_single.txt", loaded_Pairs));
    BOOST_CHECK_EQUAL(loaded_Pairs.size(), 1);
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(5, 10)) != loaded_Pairs.end());
    
    std::remove("pairsT_IO_single.txt");
}

BOOST_AUTO_TEST_CASE(read_multiple_pairs_from_same_image)
{
    // Test the file format where one line has: I J1 J2 J3 ...
    // This should create pairs (I, J1), (I, J2), (I, J3)
    std::ofstream out("pairsT_IO_multiple.txt");
    out << "0 1 2 3\n";
    out << "1 4 5\n";
    out.close();
    
    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_multiple.txt", loaded_Pairs));
    
    // With symmetry, pairs should be normalized to (min, max)
    BOOST_CHECK_EQUAL(loaded_Pairs.size(), 5);
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(0, 1)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(0, 2)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(0, 3)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(1, 4)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(1, 5)) != loaded_Pairs.end());
    
    std::remove("pairsT_IO_multiple.txt");
}

BOOST_AUTO_TEST_CASE(duplicate_pairs_are_handled)
{
    // Test that duplicate pairs (after symmetry normalization) are handled correctly
    // Create a file with duplicates, including pairs that become duplicates after normalization
    std::ofstream out("pairsT_IO_dup.txt");
    out << "0 1\n";   // (0, 1)
    out << "1 0\n";   // Should normalize to (0, 1) - duplicate
    out << "0 1\n";   // (0, 1) - duplicate
    out << "2 3\n";   // (2, 3)
    out << "3 2\n";   // Should normalize to (2, 3) - duplicate
    out.close();
    
    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_dup.txt", loaded_Pairs, true));
    
    // With symmetry normalization, all duplicates should be merged
    // Should only have 2 unique pairs: (0, 1) and (2, 3)
    BOOST_CHECK_EQUAL(loaded_Pairs.size(), 2);
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(0, 1)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(2, 3)) != loaded_Pairs.end());
    
    std::remove("pairsT_IO_dup.txt");
}

BOOST_AUTO_TEST_CASE(large_image_indices)
{
    // Test with large image indices
    PairSet pairSetGT;
    pairSetGT.insert(std::make_pair(1000, 2000));
    pairSetGT.insert(std::make_pair(5000, 10000));
    pairSetGT.insert(std::make_pair(999, 1001));
    
    BOOST_CHECK(savePairsToFile("pairsT_IO_large.txt", pairSetGT));
    
    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_large.txt", loaded_Pairs));
    BOOST_CHECK_EQUAL(loaded_Pairs.size(), 3);
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(1000, 2000)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(5000, 10000)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(999, 1001)) != loaded_Pairs.end());
    
    std::remove("pairsT_IO_large.txt");
}

BOOST_AUTO_TEST_CASE(symmetry_normalization)
{
    // Test that pairs are correctly normalized with symmetry
    std::ofstream out("pairsT_IO_sym.txt");
    out << "5 3\n";  // Should become (3, 5)
    out << "1 4\n";  // Should stay (1, 4)
    out << "8 2\n";  // Should become (2, 8)
    out.close();
    
    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_sym.txt", loaded_Pairs, true));
    
    BOOST_CHECK_EQUAL(loaded_Pairs.size(), 3);
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(3, 5)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(1, 4)) != loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(2, 8)) != loaded_Pairs.end());
    
    // These should NOT be found (wrong order)
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(5, 3)) == loaded_Pairs.end());
    BOOST_CHECK(loaded_Pairs.find(std::make_pair(8, 2)) == loaded_Pairs.end());
    
    std::remove("pairsT_IO_sym.txt");
}

BOOST_AUTO_TEST_CASE(roundtrip_consistency)
{
    // Test that multiple save/load cycles preserve data
    PairSet original;
    original.insert(std::make_pair(0, 10));
    original.insert(std::make_pair(1, 11));
    original.insert(std::make_pair(2, 12));
    original.insert(std::make_pair(3, 13));
    
    // First save/load cycle
    BOOST_CHECK(savePairsToFile("pairsT_IO_round1.txt", original));
    PairSet loaded1;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_round1.txt", loaded1));
    
    // Second save/load cycle
    BOOST_CHECK(savePairsToFile("pairsT_IO_round2.txt", loaded1));
    PairSet loaded2;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO_round2.txt", loaded2));
    
    // All should be equal
    BOOST_CHECK(std::equal(original.begin(), original.end(), loaded1.begin()));
    BOOST_CHECK(std::equal(original.begin(), original.end(), loaded2.begin()));
    BOOST_CHECK(std::equal(loaded1.begin(), loaded1.end(), loaded2.begin()));
    
    std::remove("pairsT_IO_round1.txt");
    std::remove("pairsT_IO_round2.txt");
}

