// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>
#include <aliceVision/types.hpp>

#include <vector>
#include <unordered_map>

namespace aliceVision {
namespace calibration {

/**
 * @brief Detect checkerboard structures in an image.
 *
 * This class provides an interface for detecting checkerboards in a given image.
 * It provides access to the detected corners and checkerboards, each board being represented as a matrix of corner IDs.
 *
 * A corner is described by:
 * - its center
 * - its two principal dimensions
 * - a scale of detection (higher scale means better detection quality).
 *
 * Checkerboards may be:
 * - heavily distorted
 * - with large blur
 * - partially occluded (even at the center)
 * - without any size information.
 *
 * Also, the detection algorithm supports both simple and nested grids.
 * Note however that in this case the checkerboards must be centered on the image center.
 *
 * It is possible to visualize the output of the detection by either drawing it on an image or directly retrieving a "debug image".
 *
 * References: [ROCHADE], [Geiger], [Chen], [Liu], [Abdulrahman], [Bok]
 */
class CheckerDetector
{
  public:
    /**
     * @brief A checkerboard is represented as a matrix of indices, each index referencing a corner.
     */
    using CheckerBoard = Eigen::Matrix<IndexT, -1, -1>;

    /// Intermediate computation structure.
    struct NewPoint
    {
        IndexT row;
        IndexT col;
        double score;
    };

    /// Intermediate computation structure.
    struct IntermediateCorner
    {
        Vec2 center;
        double scale;

        IntermediateCorner() = default;
        IntermediateCorner(const Vec2& c, double s)
          : center(c),
            scale(s)
        {}
    };

    /**
     * @brief Entities extracted from the input images that can be connected in grid structures to form a checkerboard.
     */
    struct CheckerBoardCorner
    {
        Vec2 center;
        Vec2 dir1;
        Vec2 dir2;
        double scale;

        CheckerBoardCorner() = default;
        CheckerBoardCorner(const Vec2& c, double s)
          : center(c),
            scale(s)
        {}
        CheckerBoardCorner(const Vec2& c, const Vec2& d1, const Vec2& d2, double s)
          : center(c),
            dir1(d1),
            dir2(d2),
            scale(s)
        {}
    };

  public:
    /**
     * @brief Checkerboard detection routine.
     *
     * Algorithm steps:
     * 1. convert image to grayscale
     * 2. extract corner positions from image at different scales and merge them to avoid redundancies
     * 3. normalize image
     * 4. compute the corners directions
     * 5. build checkerboards by connecting these corners
     * 6. merge connected boards
     * 7. remove invalid boards
     * 8. if we are using nested calibration grids:
     *     8.1. sort the boards by distance to image center
     *     8.2. filter boards to only keep a sequence of nested boards
     *     8.3. connect and group these nested boards
     *
     * @param source[in] Input image containing checkerboards.
     * @param useNestedGrid[in] Indicate if the image contains nested calibration grids.
     * @param debug[in] Indicate if debug images should be drawn.
     * @return False if a problem occured during detection, otherwise true.
     */
    bool process(const image::Image<image::RGBColor>& source, bool useNestedGrids = false, bool debug = false);

    /// Return a copy of detected checkerboards.
    std::vector<CheckerBoard> getBoards() const { return _boards; }

    /// Return a copy of detected corners.
    std::vector<CheckerBoardCorner> getCorners() const { return _corners; }

    /// Return a reference to detected checkerboards.
    std::vector<CheckerBoard>& getBoards() { return _boards; }

    /// Return a reference to detected corners.
    std::vector<CheckerBoardCorner>& getCorners() { return _corners; }

    /**
     * @brief Draw detected checkerboards on an image.
     *
     * A checkerboard is visually represented by lines connecting its corners (i.e. by its edges).
     *
     * @param[in,out] img Image on which to draw the grid lines.
     * @param[in] nestedCheckers Indicate if the input image contained nested calibration grids..
     */
    void drawCheckerBoard(image::Image<image::RGBColor>& img, bool nestedCheckers = false) const;

    /// Return a reference to the debug image.
    const image::Image<image::RGBColor>& getDebugImage() const { return _debugImage; }

  private:
    /**
     * @brief Extract corners positions from the image at the given scale.
     *
     * Algorithm steps:
     * 1. apply scale and normalize the input image
     * 2. compute Hessian response of the image
     * 3. extract corners positions using the Hessian response
     * 4. refine the corners positions using the image
     * 5. prune corners
     *
     * @param[out] corners Container for extracted corners.
     * @param[in] input Input grayscale image.
     * @param[in] scale Scale applied to the image before the extraction.
     * @return False if a problem occured during extraction, otherwise true.
     */
    bool processLevel(std::vector<Vec2>& corners, const image::Image<float>& input, double scale) const;

    /**
     * @brief Retrieve min and max pixel values of a grayscale image.
     *
     * @param[out] min Minimum pixel value.
     * @param[out] max Maximum pixel value.
     * @param[in] input Input grayscale image.
     */
    void getMinMax(float& min, float& max, const image::Image<float>& input) const;

    /**
     * @brief Normalize a grayscale image so that min pixel value is 0 and max pixel value is 1.
     *
     * @param[out] output Normalized image.
     * @param[in] input Input grayscale image.
     */
    void normalizeImage(image::Image<float>& output, const image::Image<float>& input) const;

    /**
     * @brief Compute Hessian response at each pixel of an image.
     * @see [Chen]
     *
     * Hessian response formula is close to the Hessian matrix determinant absolute value,
     * but yields better results in practice.
     *
     * @param[out] output Hessian response at each pixel of input image.
     * @param[in] input Input grayscale image.
     */
    void computeHessianResponse(image::Image<float>& output, const image::Image<float>& input) const;

    /**
     * @brief Extract corner positions by searching local maxima (on a 7x7 patch) in the Hessian response.
     * @see [Liu]
     *
     * @param[out] rawCorners Corners positions.
     * @param[in] hessianResponse Hessian response of input image.
     */
    void extractCorners(std::vector<Vec2>& rawCorners, const image::Image<float>& hessianResponse) const;

    /**
     * @brief Refine corners positions using image gradient (on a 5x5 patch) around the initial positions.
     * @see [Geiger]
     * @see [Abdulrahman]
     *
     * @param[out] refinedCorners Refined corners positions.
     * @param[in] rawCorners Initial corners positions.
     * @param[in] input Input grayscale image.
     */
    void refineCorners(std::vector<Vec2>& refinedCorners, const std::vector<Vec2>& rawCorners, const image::Image<float>& input) const;

    /**
     * @brief Analyze grayscale values in the neighborhood of given corners positions and select the ones that match a checkerboard pattern.
     * @see [Bok]
     *
     * Criteria to match the pattern at a corner position:
     * - sample grayscale values on a circle of 5px radius around the position
     * - normalize these values
     * - count the number of sign changes between consecutive samples
     * - to match the pattern, this count should be equal to 4 (alterning black and white tiles around a corner).
     *
     * @param[out] prunedCorners Selected corners positions.
     * @param[in] rawCorners Initial corners positions.
     * @param[in] input Input grayscale image.
     */
    void pruneCorners(std::vector<Vec2>& prunedCorners, const std::vector<Vec2>& rawCorners, const image::Image<float>& input) const;

    /**
     * @brief Given corners positions, refine their positions and compute their directions.
     * @see [ROCHADE]
     * @see [Geiger]
     *
     * @param[out] refinedCorners Container for corners with computed directions.
     * @param[in] rawCorners Corners positions.
     * @param[in] input Input grayscale image.
     */
    void fitCorners(std::vector<CheckerBoardCorner>& refinedCorners,
                    const std::vector<IntermediateCorner>& rawCorners,
                    const image::Image<float>& input) const;

    /**
     * @brief Build checkerboards by connecting corners.
     * @see [Geiger]
     *
     * Algorithm for building a checkerboard:
     * 1. find a suitable seed corner and initialize the board with it
     * 2. while it is possible, perform a grow step on the board: find corners that can be connected to the board and include them
     * 3. compute the board's energy and reject it if it is above a certain threshold
     *
     * @param[out] boards Container for the built checkerboards.
     * @param[in] refinedCorners Corners with directions information.
     * @param[in] input Input grayscale image.
     */
    void buildCheckerboards(std::vector<CheckerBoard>& boards,
                            const std::vector<CheckerBoardCorner>& refinedCorners,
                            const image::Image<float>& input) const;

    /**
     * @brief Find corner closest to a given position in an area constrained to a small cone around a given direction.
     *
     * @param[in] center Input position.
     * @param[in] dir Input direction in which to look for a corner.
     * @param[in] refinedCorners Array of corners.
     * @return Index of closest corner in the input array, UndefinedIndexT if none was found.
     */
    IndexT findClosestCorner(const Vec2& center, const Vec2& dir, const std::vector<CheckerBoardCorner>& refinedCorners) const;

    /**
     * @brief Check if a given corner can be used as seed for checkerboard detection.
     *
     * @param[out] board Initialized board if the corner can be used as a seed.
     * @param[in] seed Index of the corner to check.
     * @param[in] refinedCorners Array of corners.
     * @return True if the corner can be used as a seed, otherwise false.
     */
    bool getSeedCheckerboard(CheckerBoard& board, IndexT seed, const std::vector<CheckerBoardCorner>& refinedCorners) const;

    /**
     * @brief Compute the "energy" of a checkerboard using the number of valid corners and the maximum local distortion along rows and columns.
     *
     * @param[in] board Input checkerboard.
     * @param[in] refinedCorners Checkerboard corners.
     * @return Checkerboard's energy value.
     */
    double computeEnergy(const CheckerBoard& board, const std::vector<CheckerBoardCorner>& refinedCorners) const;

    /**
     * @brief Find positions on a board that do not reference a corner yet but can be used to extend the board.
     *
     * @param[out] candidates Selected candidate points.
     * @param[in] board Input checkerboard.
     * @param[in] inside Search points for extending the board inwards or outwards.
     * @return False if the board has less than two rows, otherwise true.
     */
    bool getCandidates(std::vector<NewPoint>& candidates, const CheckerBoard& board, bool inside) const;

    /**
     * @brief Extend a board in the up direction.
     *
     * Algorithm steps:
     * 1. enlarge board in the up direction with empty indices
     * 2. find candidate points for extension
     * 3. for each candidate point, find a corner that minimizes local distortion with the board
     * (note: each corner can be associated to at most one candidate point)
     * 4. filter extended board and check that its energy is lower than before.
     *
     * @param[in,out] board Checkerboard to extend.
     * @param[in] refinedCorners All detected corners.
     * @param[in] nested Extend the board inwards or outwards.
     * @return False if a problem occured or if the energy of the extended board is higher than before, otherwise true.
     */
    bool growIterationUp(CheckerBoard& board, const std::vector<CheckerBoardCorner>& refinedCorners, bool nested) const;

    /**
     * @brief Extend a board outwards in the up, down, right and left directions (only if that creates a board with lower energy).
     *
     * @param[in,out] board Checkerboard to extend.
     * @param[in] refinedCorners All detected corners.
     * @return False if the energy of the extended board is higher than before, otherwise true.
     */
    bool growIteration(CheckerBoard& board, const std::vector<CheckerBoardCorner>& refinedCorners) const;

    /**
     * @brief Merge connected checkboards.
     *
     * Algorithm steps:
     * 1. sort checkerboards by energy
     * 2. for each board:
     *     2.1. try to find a board with higher energy, common corners and no conflict
     *     2.2. create a merged board from these two boards
     *     2.3. if the new board energy is lower than the first board energy, replace the latter
     * 3. remove overlapping boards
     *
     * @return False if a problem occured during merging, otherwise true.
     */
    bool mergeCheckerboards();

    /**
     * @brief Trim the empty borders of a checkerboard.
     *
     * @param[in] board Input checkerboard.
     * @return Same checkerboard without empty borders.
     */
    CheckerBoard trimEmptyBorders(const CheckerBoard& board) const;

    /**
     * @brief Find a common corner between two checkerboards.
     *
     * @param[in] board Input checkerboard.
     * @param[in] cornerLookup Lookup table for corners of reference checkerboard.
     * @param[out] coordsRef Coordinates of common corner in reference board.
     * @param[out] coordsBoard Coordinates of common corner in input board.
     * @return True if a common corner was found, otherwise false.
     */
    bool findCommonCorner(const CheckerBoard& board,
                          const std::unordered_map<IndexT, Vec2i>& cornerLookup,
                          Vec2i& coordsRef,
                          Vec2i& coordsBoard) const;

    /**
     * @brief Merge two checkeroards that share a common corner.
     *
     * @param[in] boardRef Reference checkerboard.
     * @param[in] board Checkerboard to merge with the reference board.
     * @param[in] coordsRef Common corner coordinates on reference board.
     * @param[in] coordsBoard Common corner coordinates on board to merge.
     * @param[out] boardMerge Resulting checkerboard after merge.
     * @return False if a conflict was detected, otherwise true.
     */
    bool mergeCheckerboardsPair(const CheckerBoard& boardRef,
                                const CheckerBoard& board,
                                const Vec2i& coordsRef,
                                const Vec2i& coordsBoard,
                                CheckerBoard& boardMerge) const;

    /**
     * @brief Filter out overlapping checkerboards.
     *
     * When two checkerboards share at least 80% of their corners, keep only the one with lowest energy.
     *
     * @param[in] checkersWithScore Checkerboards with their corresponding energy.
     * @param[out] toKeep Indices of checkerboards to keep.
     */
    void filterOverlapping(const std::vector<std::pair<CheckerBoard, double>>& checkersWithScore, std::vector<std::size_t>& toKeep) const;

    /**
     * @brief Filter out checkboards based on a geometric invalidation criteria.
     *
     * A checkboard is considered invalid if one of its rows or columns contains two edges
     * that have an absolute angle between them that is above a certain threshold (currently PI/4).
     *
     * @return False if a problem occured during filtering, otherwise true.
     */
    bool removeInvalidCheckerboards();

    /**
     * @brief Sort checkerboards using minimal distance between their corners and image center as criteria.
     *
     * @param[in] center Image center.
     */
    void sortCheckerBoardsByDistanceToCenter(const Vec2& center);

    /**
     * @brief Compute minimal distance between a center point and the corners of a checkerboard.
     *
     * @param[in] board Input checkerboard.
     * @param[in] center Center position.
     * @return Minimal distance between checkerboard and center position.
     */
    double minDistanceToCenter(const CheckerBoard& board, const Vec2& center) const;

    /**
     * @brief Keep only a sequence of nested boards and discard the other boards.
     *
     * This method assumes that checkboards are sorted by distance to center.
     *
     * Also, boards must be centered on image center to be recognized as nested.
     *
     * @param[in] height Image height.
     * @param[in] width Image width.
     */
    void filterNestedCheckerBoards(const size_t& height, const size_t& width);

    /**
     * @brief Extend boards so that nested boards will overlap.
     */
    void buildNestedConnectors();

    /**
     * @brief Merge overlapping nested boards.
     */
    void groupNestedCheckerboards();

    /**
     * @brief Merge given board with reference board (first in the nesting sequence).
     *
     * @param[in] ref_center Center point of reference board.
     * @param[in] other Index of board to merge.
     * @param[in] scale Checkboard scale within nesting sequence (power of 2).
     * @return False if merging failed, otherwise true.
     */
    bool groupNestedCheckerboardsPair(Vec2i& ref_center, const IndexT& other, int scale);

  private:
    /// Detected checkerboards.
    std::vector<CheckerBoard> _boards;

    /// Detected corners.
    std::vector<CheckerBoardCorner> _corners;

    /// Input image with detected checkerboards drawn over it.
    image::Image<image::RGBColor> _debugImage;
};

}  // namespace calibration
}  // namespace aliceVision
