// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "geometricFilterUtils.hpp"
#include <ceres/ceres.h>

namespace aliceVision {
namespace matchingImageCollection {

void copyInlierMatches(const std::vector<size_t> &inliers,
                       const matching::MatchesPerDescType &putativeMatchesPerType,
                       const std::vector<feature::EImageDescriberType> &descTypes,
                       matching::MatchesPerDescType &out_geometricInliersPerType)
{
  std::vector<size_t> orderedInliers = inliers;
  std::sort(orderedInliers.begin(), orderedInliers.end());

  size_t currentDescType = 0;
  size_t currentDescTypeStartIndex = 0;
  auto currentDescTypeMaxLength = putativeMatchesPerType.getNbMatches(descTypes[currentDescType]);

  for(const size_t globalInlierIndex : orderedInliers)
  {
    while (globalInlierIndex >= currentDescTypeMaxLength)
    {
      ++currentDescType;
      currentDescTypeStartIndex = currentDescTypeMaxLength;
      currentDescTypeMaxLength += putativeMatchesPerType.getNbMatches(descTypes[currentDescType]);
    }
    const size_t localIndex = globalInlierIndex - currentDescTypeStartIndex;
    const auto descType = descTypes[currentDescType];
    out_geometricInliersPerType[descType].push_back(putativeMatchesPerType.at(descType)[localIndex]);
  }
}

void centerMatrix(const Eigen::Matrix2Xf & points2d, Mat3 & t)
{
  t = Mat3::Identity();

  const Vec2f mean = points2d.rowwise().mean();
  const auto nbPoints = points2d.cols();

  Vec2f stdDev = ((points2d.colwise() - mean).cwiseAbs2().rowwise().sum()/(nbPoints - 1)).cwiseSqrt();

  if(stdDev(0) < 0.1)
    stdDev(0) = 0.1;
  if(stdDev(1) < 0.1)
    stdDev(1) = 0.1;

  t << 1./stdDev(0), 0.,            -mean(0)/stdDev(0),
          0.,            1./stdDev(1),  -mean(1)/stdDev(1),
          0.,            0.,            1.;
}


void centeringMatrices(const std::vector<feature::PointFeature> & featuresI,
                       const std::vector<feature::PointFeature> & featuresJ,
                       const matching::IndMatches & matches,
                       Mat3 & cI,
                       Mat3 & cJ,
                       const std::set<IndexT> & usefulMatchesId)
{
  assert(!featuresI.empty());
  assert(!featuresJ.empty());
  assert(!matches.empty());
  assert(*std::max_element(usefulMatchesId.begin(), usefulMatchesId.end()) <= matches.size()); // prevent segfault

  std::set<IndexT> matchesId = usefulMatchesId; // duplicate
  std::size_t nbMatches = usefulMatchesId.size();

  if (usefulMatchesId.empty())
  {
    nbMatches = matches.size();
    // set every match as useful for estimation
    for (IndexT i = 0; i < nbMatches; ++i)
      matchesId.insert(i);
  }

  int iMatch = 0;
  Matf ptsI(2, nbMatches);
  Matf ptsJ(2, nbMatches);

  for (IndexT matchId : matchesId)
  {
    ptsI.col(iMatch) = featuresI.at(matches.at(matchId)._i).coords();
    ptsJ.col(iMatch) = featuresJ.at(matches.at(matchId)._j).coords();
    ++iMatch;
  }

  centerMatrix(ptsI, cI);
  centerMatrix(ptsJ, cJ);
}

void computeSimilarity(const feature::PointFeature & feat1,
                       const feature::PointFeature & feat2,
                       Mat3 & S)
{
  S = Mat3::Identity();

  const Vec2f & coord1 = feat1.coords();
  const double scale1 = feat1.scale();
  const double orientation1 = feat1.orientation();

  const Vec2f & coord2 = feat2.coords();
  const double scale2 =  feat2.scale();
  const double orientation2 = feat2.orientation();

  const double c1 = cos(orientation1);
  const double s1 = sin(orientation1);
  const double c2 = cos(orientation2);
  const double s2 = sin(orientation2);

  Mat3 A1;
  A1 << scale1 * c1, scale1 * (-s1), coord1(0),
          scale1 * s1, scale1 * c1, coord1(1),
          0, 0, 1;
  Mat3 A2;
  A2 << scale2 * c2, scale2 * (-s2), coord2(0),
          scale2 * s2, scale2 * c2, coord2(1),
          0, 0, 1;

  S = A2 * A1.inverse();
}

void estimateAffinity(const std::vector<feature::PointFeature> & featuresI,
                      const std::vector<feature::PointFeature> & featuresJ,
                      const matching::IndMatches & matches,
                      Mat3 & affineTransformation,
                      const std::set<IndexT> & usefulMatchesId)
{
  assert(!featuresI.empty());
  assert(!featuresJ.empty());
  assert(!matches.empty());
  assert(*std::max_element(usefulMatchesId.begin(), usefulMatchesId.end()) <= matches.size()); // prevent segfault

  affineTransformation = Mat3::Identity();

  std::set<IndexT> matchesId = usefulMatchesId; // duplicate

  std::size_t nbMatches = usefulMatchesId.size();

  if (usefulMatchesId.empty())
  {
    nbMatches = matches.size();
    // set every match as useful for estimation
    for (IndexT i = 0; i < nbMatches; ++i)
      matchesId.insert(i);
  }

  Mat M(Mat::Zero(2*nbMatches,6));
  Vec b(2*nbMatches);
  int iMatch = 0;
  for (IndexT matchId : matchesId)
  {
    const feature::PointFeature & featI = featuresI.at(matches.at(matchId)._i);
    const feature::PointFeature & featJ = featuresJ.at(matches.at(matchId)._j);
    const Vec2 featICoords (featI.x(), featI.y());

    M.block(iMatch,0,1,3) = featICoords.homogeneous().transpose();
    M.block(iMatch+nbMatches,3,1,3) = featICoords.homogeneous().transpose();
    b(iMatch) = featJ.x();
    b(iMatch+nbMatches) = featJ.y();

    ++iMatch;
  }

  const Vec a = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  affineTransformation.row(0) = a.topRows(3).transpose();
  affineTransformation.row(1) = a.bottomRows(3).transpose();
  affineTransformation(2,0) = 0.;
  affineTransformation(2,1) = 0.;
  affineTransformation(2,2) = 1.;
}

void estimateHomography(const std::vector<feature::PointFeature> &featuresI,
                        const std::vector<feature::PointFeature> &featuresJ,
                        const matching::IndMatches &matches,
                        Mat3 &H,
                        const std::set<IndexT> &usefulMatchesId)
{
  assert(!featuresI.empty());
  assert(!featuresJ.empty());
  assert(!matches.empty());
  assert(*std::max_element(usefulMatchesId.begin(), usefulMatchesId.end()) <= matches.size()); // prevent segfault

  H = Mat3::Identity();

  std::size_t nbMatches = usefulMatchesId.size();

  std::set<IndexT> matchesId = usefulMatchesId; // duplicate

  if (usefulMatchesId.empty())
  {
    nbMatches = matches.size();
    // set every match as useful for estimation
    for (IndexT i = 0; i < nbMatches; ++i)
      matchesId.insert(i);
  }

  Mat3 CI, CJ;
  centeringMatrices(featuresI, featuresJ, matches, CI, CJ, matchesId);

  Mat A(Mat::Zero(2*nbMatches,9));

  IndexT iMatch = 0;
  for(IndexT matchId : matchesId)
  {
    const feature::PointFeature & featI = featuresI.at(matches.at(matchId)._i);
    const feature::PointFeature & featJ = featuresJ.at(matches.at(matchId)._j);
    Vec2 fI(featI.x(), featI.y());
    Vec2 fJ(featJ.x(), featJ.y());
    Vec3 ptI = CI * fI.homogeneous();
    Vec3 ptJ = CJ * fJ.homogeneous();

    A.block(iMatch,0,1,3) = ptI.transpose();
    A.block(iMatch,6,1,3) = -ptJ(0) * ptI.transpose();
    A.block(iMatch+nbMatches,3,1,3) = ptI.transpose();
    A.block(iMatch+nbMatches,6,1,3) = -ptJ(1) * ptI.transpose();
    ++iMatch;
  }

  Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  Vec h = svd.matrixV().rightCols(1);
  Mat3 H0;
  H0.row(0) = h.topRows(3).transpose();
  H0.row(1) = h.middleRows(3,3).transpose();
  H0.row(2) = h.bottomRows(3).transpose();

  H = CJ.inverse() * H0 * CI;
  if(std::fabs(H(2, 2)) > std::numeric_limits<double>::epsilon())
    H /= H(2,2);
}

void findTransformationInliers(const std::vector<feature::PointFeature> &featuresI,
                               const std::vector<feature::PointFeature> &featuresJ,
                               const matching::IndMatches &matches,
                               const Mat3 &transformation,
                               double tolerance,
                               std::set<IndexT> &inliersId)
{
  inliersId.clear();
  const double squaredTolerance = Square(tolerance);

#pragma omp parallel for
  for (int iMatch = 0; iMatch < matches.size(); ++iMatch)
  {
    const feature::PointFeature & featI = featuresI.at(matches.at(iMatch)._i);
    const feature::PointFeature & featJ = featuresJ.at(matches.at(iMatch)._j);

    const Vec2 ptI(featI.x(), featI.y());
    const Vec2 ptJ(featJ.x(), featJ.y());

    const Vec3 ptIp_hom = transformation * ptI.homogeneous();

    const double dist = (ptJ - ptIp_hom.hnormalized()).squaredNorm();

    if (dist < squaredTolerance)
    {
#pragma omp critical
      inliersId.insert(iMatch);
    }
  }
}

void findTransformationInliers(const Mat2X& featuresI,
                               const Mat2X& featuresJ,
                               const matching::IndMatches &matches,
                               const Mat3 &transformation,
                               double tolerance,
                               std::set<IndexT> &inliersId)
{
  inliersId.clear();
  const double squaredTolerance = Square(tolerance);

#pragma omp parallel for
  for (int iMatch = 0; iMatch < matches.size(); ++iMatch)
  {
    const matching::IndMatch& match = matches.at(iMatch);
    const Vec2 & ptI = featuresI.col(match._i);
    const Vec2 & ptJ = featuresJ.col(match._j);

    const Vec3 ptIp_hom = transformation * ptI.homogeneous();

    const double dist = (ptJ - ptIp_hom.hnormalized()).squaredNorm();

    if (dist < squaredTolerance)
    {
#pragma omp critical
      inliersId.insert(iMatch);
    }
  }
}

/**
 * @brief This functor allows to optimize an Homography.
 * @details It is based on [F.Srajer, 2016] p.20, 21 and its C++ implementation: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L992
 * "The optimization takes into account points with error close to the threshold and does not care about high-error ones."
 */
class RefineHRobustCostFunctor
{
public:

    RefineHRobustCostFunctor(const Vec2& x1, const Vec2& x2,
                             double softThresh)
            : x1(x1),x2(x2),_softThresh(softThresh)
    {
    }

    template<typename T>
    bool operator()(const T* const parameters, T* residuals) const
    {
      using Mat3T = Eigen::Matrix<T, 3, 3>;
      using Vec3T = Eigen::Matrix<T, 3, 1>;
      using Vec2T = Eigen::Matrix<T, 2, 1>;

      const Vec2T x(T(x1(0)), T(x1(1)));
      const Vec2T y(T(x2(0)), T(x2(1)));

      const Mat3T H(parameters);

      const Vec3T xp = H * x.homogeneous();

      const T errX = y(0) - xp(0)/xp(2);
      const T errY = y(1) - xp(1)/xp(2);
      const T errSq = errX*errX + errY*errY;

      // Avoid division by zero in derivatives computation
      const T err = (errSq==0.) ? T(errSq) : T(sqrt(errSq));
      residuals[0] = robustify(_softThresh, err);

      return true;
    }

    template<typename T>
    /**
     * @brief robustify
     * Based on: https://github.com/fsrajer/yasfm/blob/3a09bc0ee69b7021910d646386cd92deab504a2c/YASFM/utils.h#L347
     * @param softThresh
     * @param x
     * @return
     */
    static T robustify(double softThresh, T x)
    {
      const double t = 0.25;
      const double sigma = softThresh / std::sqrt(-std::log(t * t));

      return -ceres::log(ceres::exp(-(x * x) / T(2.0 * sigma * sigma)) + T(t)) + T(std::log(1.0 + t));
    }

    Vec2 x1, x2;
    double _softThresh;
};

bool refineHomography(const std::vector<feature::PointFeature> &featuresI,
                      const std::vector<feature::PointFeature> &featuresJ,
                      const matching::IndMatches& remainingMatches,
                      Mat3& homography,
                      std::set<IndexT>& bestMatchesId,
                      double homographyTolerance)
{
  Mat2X pointsI;
  Mat2X pointsJ;
  feature::PointsToMat(featuresI, pointsI);
  feature::PointsToMat(featuresJ, pointsJ);
  return refineHomography(pointsI,
                          pointsJ,
                          remainingMatches,
                          homography,
                          bestMatchesId,
                          homographyTolerance);
}

bool refineHomography(const Mat2X& features_I,
                      const Mat2X& features_J,
                      const matching::IndMatches& remainingMatches,
                      Mat3& homography,
                      std::set<IndexT>& bestMatchesId,
                      double homographyTolerance)
{
  ceres::Problem problem;
  // use a copy for the optimization to avoid changes in the input one
  Mat3 tempHomography = homography;

  for(IndexT matchId : bestMatchesId)
  {
    const matching::IndMatch& match = remainingMatches.at(matchId);

    const Vec2& x1 = features_I.col(match._i);
    const Vec2& x2 = features_J.col(match._j);

    RefineHRobustCostFunctor* costFun =
            new RefineHRobustCostFunctor(x1, x2, homographyTolerance);

    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                    RefineHRobustCostFunctor,
                    1,
                    9>(costFun),
            nullptr,
            tempHomography.data());
  }

  ceres::Solver::Options solverOpt;
  solverOpt.max_num_iterations = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(solverOpt,&problem,&summary);

  // if the optimization did not succeed return without changing
  if(!summary.IsSolutionUsable())
    return false;

  homography = tempHomography;

  // normalize the homography
  if(std::fabs(homography(2, 2)) > std::numeric_limits<double>::epsilon())
    homography /= homography(2,2);

  findTransformationInliers(features_I, features_J, remainingMatches, homography, homographyTolerance, bestMatchesId);

  return true;
}
}
}
