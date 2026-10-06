// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/calibration/checkerDetector.hpp>
#include <aliceVision/calibration/checkerDetector_io.hpp>

#define BOOST_TEST_MODULE checkerDetectorIO
#include <boost/test/unit_test.hpp>
#include <aliceVision/unitTest.hpp>

#include <boost/json.hpp>

using namespace aliceVision;
using namespace aliceVision::calibration;

namespace {

CheckerDetector makeDetector()
{
    CheckerDetector detector;

    std::vector<CheckerDetector::CheckerBoardCorner>& corners = detector.getCorners();
    corners.emplace_back(Vec2(10.25, 20.5), Vec2(1.0, 0.0), Vec2(0.0, 1.0), 0.0);
    corners.emplace_back(Vec2(51.125, 19.75), Vec2(0.996, 0.087), Vec2(-0.087, 0.996), 1.0);
    corners.emplace_back(Vec2(90.0, 21.0), Vec2(0.7071, 0.7071), Vec2(-0.7071, 0.7071), 2.0);
    corners.emplace_back(Vec2(11.0, 61.0), Vec2(0.1, 0.9), Vec2(0.9, -0.1), 0.5);
    corners.emplace_back(Vec2(1.0 / 3.0, 1e-7), Vec2(-1.0, 0.0), Vec2(0.0, -1.0), 3.0);

    // Non square board with a missing corner
    CheckerDetector::CheckerBoard board(2, 3);
    board << 0, 1, 2, 3, UndefinedIndexT, 4;
    detector.getBoards().push_back(board);

    CheckerDetector::CheckerBoard single(1, 1);
    single << 4;
    detector.getBoards().push_back(single);

    return detector;
}

void checkEqual(const CheckerDetector& expected, const CheckerDetector& actual)
{
    const auto expectedCorners = expected.getCorners();
    const auto actualCorners = actual.getCorners();
    BOOST_REQUIRE_EQUAL(actualCorners.size(), expectedCorners.size());
    for (std::size_t i = 0; i < expectedCorners.size(); ++i)
    {
        BOOST_CHECK_EQUAL(actualCorners[i].center, expectedCorners[i].center);
        BOOST_CHECK_EQUAL(actualCorners[i].dir1, expectedCorners[i].dir1);
        BOOST_CHECK_EQUAL(actualCorners[i].dir2, expectedCorners[i].dir2);
        BOOST_CHECK_EQUAL(actualCorners[i].scale, expectedCorners[i].scale);
    }

    const auto expectedBoards = expected.getBoards();
    const auto actualBoards = actual.getBoards();
    BOOST_REQUIRE_EQUAL(actualBoards.size(), expectedBoards.size());
    for (std::size_t i = 0; i < expectedBoards.size(); ++i)
    {
        BOOST_REQUIRE_EQUAL(actualBoards[i].rows(), expectedBoards[i].rows());
        BOOST_REQUIRE_EQUAL(actualBoards[i].cols(), expectedBoards[i].cols());
        BOOST_CHECK(actualBoards[i] == expectedBoards[i]);
    }
}

}  // namespace

// Test summary:
// - Build a detector by hand: 5 corners, a 2x3 board with a missing corner (UndefinedIndexT) and a 1x1 board
// - Convert it to a boost::json value and back
// - Check that all corner fields (center, dir1, dir2, scale) and all boards (size, indices) are identical
BOOST_AUTO_TEST_CASE(checkerDetectorIO_roundTrip_value)
{
    const CheckerDetector detector = makeDetector();

    const boost::json::value jv = boost::json::value_from(detector);
    const CheckerDetector loaded = boost::json::value_to<CheckerDetector>(jv);

    checkEqual(detector, loaded);
}

// Test summary:
// - Same detector as checkerDetectorIO_roundTrip_value
// - Serialize it to a JSON string and parse it back (the path used between checkerboardDetection and distortionCalibration)
// - Check that all corners and boards are identical
BOOST_AUTO_TEST_CASE(checkerDetectorIO_roundTrip_serialized)
{
    // Same path as checkerboardDetection writing and distortionCalibration reading the files
    const CheckerDetector detector = makeDetector();

    const std::string text = boost::json::serialize(boost::json::value_from(detector));
    const CheckerDetector loaded = boost::json::value_to<CheckerDetector>(boost::json::parse(text));

    checkEqual(detector, loaded);
}

// Test summary:
// - Parse a hand written JSON board of 2 rows and 3 columns with data [0, 1, 2, 3, 4, 5]
// - Check that the board data is read row by row (board(0, 2) == 2, board(1, 0) == 3)
BOOST_AUTO_TEST_CASE(checkerDetectorIO_boardLayout_rowMajor)
{
    // The file format stores boards row by row
    const boost::json::value jv = boost::json::parse(R"({
        "corners": [],
        "boards": [{"rows": 2, "cols": 3, "data": [0, 1, 2, 3, 4, 5]}]
    })");

    const CheckerDetector loaded = boost::json::value_to<CheckerDetector>(jv);

    BOOST_REQUIRE_EQUAL(loaded.getBoards().size(), 1);
    const CheckerDetector::CheckerBoard board = loaded.getBoards()[0];
    BOOST_REQUIRE_EQUAL(board.rows(), 2);
    BOOST_REQUIRE_EQUAL(board.cols(), 3);
    BOOST_CHECK_EQUAL(board(0, 2), 2);
    BOOST_CHECK_EQUAL(board(1, 0), 3);
}
