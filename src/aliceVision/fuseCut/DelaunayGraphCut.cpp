// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// To analyse history of intersected geometries during vote
// #define ALICEVISION_DEBUG_VOTE

#include "DelaunayGraphCut.hpp"
// #include <aliceVision/fuseCut/MaxFlow_CSR.hpp>
#include <aliceVision/fuseCut/MaxFlow_AdjList.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/jetColorMap.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Universe.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include "nanoflann.hpp"

#include <geogram/points/kd_tree.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <random>
#include <stdexcept>

#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/progress.hpp>

namespace aliceVision {
namespace fuseCut {

namespace bfs = boost::filesystem;

// #define USE_GEOGRAM_KDTREE 1

#ifdef USE_GEOGRAM_KDTREE
#else

// static const std::size_t MAX_LEAF_ELEMENTS = 64;
static const std::size_t MAX_LEAF_ELEMENTS = 10;

struct PointVectorAdaptator
{
    using Derived = PointVectorAdaptator; //!< In this case the dataset class is myself.
    using T = double;

    const std::vector<Point3d>& _data;
    PointVectorAdaptator(const std::vector<Point3d>& data)
        : _data(data)
    {}


    /// CRTP helper method
    inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
    /// CRTP helper method
    inline       Derived& derived()       { return *static_cast<Derived*>(this); }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        return _data.at(idx).m[dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointVectorAdaptator>,
    PointVectorAdaptator,
    3 /* dim */
    > KdTree;

/**
 * A result-set class used when performing a radius based search.
 */
template <typename DistanceType, typename IndexType = size_t>
class SmallerPixSizeInRadius
{
public:
    const DistanceType radius;

    const std::vector<double>& m_pixSizePrepare;
    const std::vector<float>& m_simScorePrepare;
    size_t m_result = 0;
    const int m_i;
    bool found = false;

    inline SmallerPixSizeInRadius(DistanceType radius_,
                                  const std::vector<double>& pixSizePrepare,
                                  const std::vector<float>& simScorePrepare,
                                  int i)
        : radius(radius_)
        , m_pixSizePrepare(pixSizePrepare)
        , m_simScorePrepare(simScorePrepare)
        , m_i(i)
    {
        init();
    }

    inline void init() { clear(); }
    inline void clear() { m_result = 0; }

    inline size_t size() const { return m_result; }

    inline bool full() const { return found; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are sufficient
     */
    inline bool addPoint(DistanceType dist, IndexType index)
    {
        if(dist < radius)
        {
            ++m_result;
            if(m_simScorePrepare[index] * m_pixSizePrepare[index] * m_pixSizePrepare[index] < m_simScorePrepare[m_i] * m_pixSizePrepare[m_i] * m_pixSizePrepare[m_i])
            {
                found = true;
                return false;
            }
        }
        return true;
    }

    inline DistanceType worstDist() const { return radius; }
};
#endif


/// Filter by pixSize
void filterByPixSize(const std::vector<Point3d>& verticesCoordsPrepare, std::vector<double>& pixSizePrepare, double pixSizeMarginCoef, std::vector<float>& simScorePrepare)
{
#ifdef USE_GEOGRAM_KDTREE
    ALICEVISION_LOG_INFO("Build geogram KdTree index.");
    GEO::AdaptiveKdTree kdTree(3);
    kdTree.set_exact(false);
    kdTree.set_points(verticesCoordsPrepare.size(), verticesCoordsPrepare[0].m);
#else
    ALICEVISION_LOG_INFO("Build nanoflann KdTree index.");
    PointVectorAdaptator pointCloudRef(verticesCoordsPrepare);
    KdTree kdTree(3 /*dim*/, pointCloudRef, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
    kdTree.buildIndex();
#endif
    ALICEVISION_LOG_INFO("KdTree created for " << verticesCoordsPrepare.size() << " points.");

    #pragma omp parallel for
    for(int vIndex = 0; vIndex < verticesCoordsPrepare.size(); ++vIndex)
    {
        if(pixSizePrepare[vIndex] == -1.0)
        {
            continue;
        }
        const double pixSizeScore = pixSizeMarginCoef * simScorePrepare[vIndex] * pixSizePrepare[vIndex] * pixSizePrepare[vIndex];
        if(pixSizeScore < std::numeric_limits<double>::epsilon())
        {
            pixSizePrepare[vIndex] = -1.0;
            continue;
        }
#ifdef USE_GEOGRAM_KDTREE
        static const std::size_t nbNeighbors = 20;
        static const double nbNeighborsInv = 1.0 / (double)nbNeighbors;
        std::array<GEO::index_t, nbNeighbors> nnIndex;
        std::array<double, nbNeighbors> sqDist;
        // kdTree.get_nearest_neighbors(nbNeighbors, verticesCoordsPrepare[i].m, &nnIndex.front(), &sqDist.front());
        kdTree.get_nearest_neighbors(nbNeighbors, vIndex, &nnIndex.front(), &sqDist.front());

        for(std::size_t n = 0; n < nbNeighbors; ++n)
        {
            // NOTE: we don't need to test the distance regarding pixSizePrepare[nnIndex[vIndex]]
            //       as we kill ourself only if our pixSize is bigger
            if(sqDist[n] < pixSizeScore)
            {
                if(pixSizePrepare[nnIndex[n]] < pixSizePrepare[vIndex] ||
                   (pixSizePrepare[nnIndex[n]] == pixSizePrepare[vIndex] && nnIndex[n] < vIndex)
                   )
                {
                    // Kill itself if inside our volume (defined by marginCoef*pixSize) there is another point with a smaller pixSize
                    pixSizePrepare[vIndex] = -1.0;
                    break;
                }
            }
            // else
            // {
            //     break;
            // }
        }
#else

        static const nanoflann::SearchParams searchParams(32, 0, false); // false: dont need to sort
        SmallerPixSizeInRadius<double, std::size_t> resultSet(pixSizeScore, pixSizePrepare, simScorePrepare, vIndex);
        kdTree.findNeighbors(resultSet, verticesCoordsPrepare[vIndex].m, searchParams);
        if(resultSet.found)
            pixSizePrepare[vIndex] = -1.0;
#endif
    }
    ALICEVISION_LOG_INFO("Filtering done.");
}


/// Remove invalid points based on invalid pixSize
void removeInvalidPoints(std::vector<Point3d>& verticesCoordsPrepare, std::vector<double>& pixSizePrepare, std::vector<float>& simScorePrepare)
{
    std::vector<Point3d> verticesCoordsTmp;
    verticesCoordsTmp.reserve(verticesCoordsPrepare.size());
    std::vector<double> pixSizeTmp;
    pixSizeTmp.reserve(pixSizePrepare.size());
    std::vector<float> simScoreTmp;
    simScoreTmp.reserve(simScorePrepare.size());
    for(int i = 0; i < verticesCoordsPrepare.size(); ++i)
    {
        if(pixSizePrepare[i] != -1.0)
        {
            verticesCoordsTmp.push_back(verticesCoordsPrepare[i]);
            pixSizeTmp.push_back(pixSizePrepare[i]);
            simScoreTmp.push_back(simScorePrepare[i]);
        }
    }
    ALICEVISION_LOG_INFO((verticesCoordsPrepare.size() - verticesCoordsTmp.size()) << " invalid points removed.");
    verticesCoordsPrepare.swap(verticesCoordsTmp);
    pixSizePrepare.swap(pixSizeTmp);
    simScorePrepare.swap(simScoreTmp);
}

void removeInvalidPoints(std::vector<Point3d>& verticesCoordsPrepare, std::vector<double>& pixSizePrepare, std::vector<float>& simScorePrepare, std::vector<GC_vertexInfo>& verticesAttrPrepare)
{
    std::vector<Point3d> verticesCoordsTmp;
    verticesCoordsTmp.reserve(verticesCoordsPrepare.size());
    std::vector<double> pixSizeTmp;
    pixSizeTmp.reserve(pixSizePrepare.size());
    std::vector<float> simScoreTmp;
    simScoreTmp.reserve(simScorePrepare.size());
    std::vector<GC_vertexInfo> verticesAttrTmp;
    verticesAttrTmp.reserve(verticesAttrPrepare.size());
    for(int i = 0; i < verticesCoordsPrepare.size(); ++i)
    {
        if(pixSizePrepare[i] != -1.0)
        {
            verticesCoordsTmp.push_back(verticesCoordsPrepare[i]);
            pixSizeTmp.push_back(pixSizePrepare[i]);
            simScoreTmp.push_back(simScorePrepare[i]);
            verticesAttrTmp.push_back(verticesAttrPrepare[i]);
        }
    }
    ALICEVISION_LOG_INFO((verticesCoordsPrepare.size() - verticesCoordsTmp.size()) << " invalid points removed.");
    verticesCoordsPrepare.swap(verticesCoordsTmp);
    pixSizePrepare.swap(pixSizeTmp);
    simScorePrepare.swap(simScoreTmp);
    verticesAttrPrepare.swap(verticesAttrTmp);
}

void createVerticesWithVisibilities(const StaticVector<int>& cams, std::vector<Point3d>& verticesCoordsPrepare, std::vector<double>& pixSizePrepare, std::vector<float>& simScorePrepare,
                                    std::vector<GC_vertexInfo>& verticesAttrPrepare, mvsUtils::MultiViewParams& mp, float simFactor, float voteMarginFactor, float contributeMarginFactor, float simGaussianSize)
{
#ifdef USE_GEOGRAM_KDTREE
    GEO::AdaptiveKdTree kdTree(3);
    kdTree.set_points(verticesCoordsPrepare.size(), verticesCoordsPrepare[0].m);
    ALICEVISION_LOG_INFO("GEOGRAM: KdTree created");
#else
    PointVectorAdaptator pointCloudRef(verticesCoordsPrepare);
    KdTree kdTree(3 /*dim*/, pointCloudRef, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
    kdTree.buildIndex();
    ALICEVISION_LOG_INFO("NANOFLANN: KdTree created.");
#endif
    // TODO FACA: update into new data structures
    // std::vector<Point3d> newVerticesCoordsPrepare(verticesCoordsPrepare.size());
    // std::vector<float> newSimScorePrepare(simScorePrepare.size());
    // std::vector<double> newPixSizePrepare(pixSizePrepare.size());
    std::vector<omp_lock_t> locks(verticesCoordsPrepare.size());
    for (auto& lock: locks)
        omp_init_lock(&lock);

    omp_set_nested(1);
    #pragma omp parallel for num_threads(3)
    for(int c = 0; c < cams.size(); ++c)
    {
        ALICEVISION_LOG_INFO("Create visibilities (" << c << "/" << cams.size() << ")");
        std::vector<float> depthMap;
        std::vector<float> simMap;
        int width, height;
        {
            const std::string depthMapFilepath = getFileNameFromIndex(mp, c, mvsUtils::EFileType::depthMap, 0);
            imageIO::readImage(depthMapFilepath, width, height, depthMap, imageIO::EImageColorSpace::NO_CONVERSION);
            if(depthMap.empty())
            {
                ALICEVISION_LOG_WARNING("Empty depth map: " << depthMapFilepath);
                continue;
            }
            int wTmp, hTmp;
            const std::string simMapFilepath = getFileNameFromIndex(mp, c, mvsUtils::EFileType::simMap, 0);
            // If we have a simMap in input use it,
            // else init with a constant value.
            if(boost::filesystem::exists(simMapFilepath))
            {
                imageIO::readImage(simMapFilepath, wTmp, hTmp, simMap, imageIO::EImageColorSpace::NO_CONVERSION);
                if(wTmp != width || hTmp != height)
                    throw std::runtime_error("Similarity map size doesn't match the depth map size: " + simMapFilepath +
                                             ", " + depthMapFilepath);
                {
                    std::vector<float> simMapTmp(simMap.size());
                    imageAlgo::convolveImage(width, height, simMap, simMapTmp, "gaussian", simGaussianSize,
                                             simGaussianSize);
                    simMap.swap(simMapTmp);
                }
            }
            else
            {
                ALICEVISION_LOG_WARNING("simMap file can't be found.");
                simMap.resize(width * height, -1);
            }

        }
        // Add visibility
        #pragma omp parallel for
        for(int y = 0; y < height; ++y)
        {
            for(int x = 0; x < width; ++x)
            {
                const std::size_t index = y * width + x;
                const float depth = depthMap[index];
                if(depth <= 0.0f)
                    continue;

                const Point3d p = mp.backproject(c, Point2d(x, y), depth);
                const double pixSize = mp.getCamPixelSize(p, c);
#ifdef USE_GEOGRAM_KDTREE
                const std::size_t nearestVertexIndex = kdTree.get_nearest_neighbor(p.m);
                // NOTE: Could compute the distance between the line (camera to pixel) and the nearestVertex OR
                //       the distance between the back-projected point and the nearestVertex
                const double dist = (p - verticesCoordsPrepare[nearestVertexIndex]).size2();
#else
                nanoflann::KNNResultSet<double, std::size_t> resultSet(1);
                std::size_t nearestVertexIndex = std::numeric_limits<std::size_t>::max();
                double dist = std::numeric_limits<double>::max();
                resultSet.init(&nearestVertexIndex, &dist);
                if(!kdTree.findNeighbors(resultSet, p.m, nanoflann::SearchParams()))
                {
                    ALICEVISION_LOG_TRACE("Failed to find Neighbors.");
                    continue;
                }
#endif
                const float pixSizeScoreI = simScorePrepare[nearestVertexIndex] * pixSize * pixSize;
                const float pixSizeScoreV = simScorePrepare[nearestVertexIndex] * pixSizePrepare[nearestVertexIndex] * pixSizePrepare[nearestVertexIndex];

                if(dist < voteMarginFactor * std::max(pixSizeScoreI, pixSizeScoreV))
                {
                    GC_vertexInfo& va = verticesAttrPrepare[nearestVertexIndex];
                    Point3d& vc = verticesCoordsPrepare[nearestVertexIndex];
                    const float simValue = simMap[index];
                    // remap similarity values from [-1;+1] to [+1;+simFactor]
                    // interpretation is [goodSimilarity;badSimilarity]
                    const float simScore = simValue < -1.0f ? 1.0f : 1.0f + (1.0f + simValue) * simFactor;

                    // Custom locks to limit it to the index: nearestVertexIndex
                    // to avoid using "omp critical"
                    omp_lock_t* lock = &locks[nearestVertexIndex];
                    omp_set_lock(lock);
                    {
                        va.cams.push_back_distinct(c);
                        if(dist < contributeMarginFactor * pixSizeScoreV)
                        {
                            vc = (vc * (double)va.nrc + p) / double(va.nrc + 1);
//                            newVerticesCoordsPrepare[nearestVertexIndex] = (newVerticesCoordsPrepare[nearestVertexIndex] * double(va.nrc) + p) / double(va.nrc + 1);
//                            newSimScorePrepare[nearestVertexIndex] = (newSimScorePrepare[nearestVertexIndex] * float(va.nrc) + simScore) / float(va.nrc + 1);
//                            newPixSizePrepare[nearestVertexIndex] = (newPixSizePrepare[nearestVertexIndex] * double(va.nrc) + pixSize) / double(va.nrc + 1);
                            va.nrc += 1;
                        }
                    }
                    omp_unset_lock(lock);
                }
            }
        }
    }
    omp_set_nested(0);

    for(auto& lock: locks)
        omp_destroy_lock(&lock);

    // compute pixSize
    #pragma omp parallel for
    for(int vi = 0; vi < verticesAttrPrepare.size(); ++vi)
    {
        GC_vertexInfo& v = verticesAttrPrepare[vi];
        v.pixSize = mp.getCamsMinPixelSize(verticesCoordsPrepare[vi], v.cams);
    }

//    verticesCoordsPrepare.swap(newVerticesCoordsPrepare);
//    simScorePrepare.swap(newSimScorePrepare);
//    pixSizePrepare.swap(newPixSizePrepare);
    ALICEVISION_LOG_INFO("Visibilities created.");
}


void DelaunayGraphCut::IntersectionHistory::append(const GeometryIntersection& geom, const Point3d& intersectPt)
{
    ++steps;
    geometries.push_back(geom);
    intersectPts.push_back(intersectPt);
    const Point3d toCam = cam - intersectPt;
    vecToCam.push_back(toCam);
    distToCam.push_back(toCam.size());
    angleToCam.push_back(angleBetwV1andV2(dirVect, intersectPt - originPt));
}

DelaunayGraphCut::DelaunayGraphCut(mvsUtils::MultiViewParams& mp)
  : _mp(mp)
{
    _camsVertexes.resize(_mp.ncams, -1);

    saveTemporaryBinFiles = _mp.userParams.get<bool>("LargeScale.saveTemporaryBinFiles", false);

    GEO::initialize();
    _tetrahedralization = GEO::Delaunay::create(3, "BDEL");
    // _tetrahedralization->set_keeps_infinite(true);
    _tetrahedralization->set_stores_neighbors(true);
    // _tetrahedralization->set_stores_cicl(true);
}

DelaunayGraphCut::~DelaunayGraphCut()
{
}

void DelaunayGraphCut::saveDhInfo(const std::string& fileNameInfo)
{
    FILE* f = fopen(fileNameInfo.c_str(), "wb");

    int npts = getNbVertices();
    fwrite(&npts, sizeof(int), 1, f);
    for(const GC_vertexInfo& v: _verticesAttr)
    {
        v.fwriteinfo(f);
    }

    int ncells = _cellsAttr.size();
    fwrite(&ncells, sizeof(int), 1, f);
    for(const GC_cellInfo& c: _cellsAttr)
    {
        c.fwriteinfo(f);
    }
    fclose(f);
}

void DelaunayGraphCut::saveDh(const std::string& fileNameDh, const std::string& fileNameInfo)
{
    ALICEVISION_LOG_DEBUG("Saving triangulation.");

    saveDhInfo(fileNameInfo);

    long t1 = clock();

    // std::ofstream oFileT(fileNameDh.c_str());
    // oFileT << *_tetrahedralization; // TODO GEOGRAM

    mvsUtils::printfElapsedTime(t1);
}


std::vector<DelaunayGraphCut::CellIndex> DelaunayGraphCut::getNeighboringCellsByGeometry(const GeometryIntersection& g) const
{
    switch (g.type)
    {
    case EGeometryType::Edge:
        return getNeighboringCellsByEdge(g.edge);
    case EGeometryType::Vertex:
        return getNeighboringCellsByVertexIndex(g.vertexIndex);
    case EGeometryType::Facet:
        return getNeighboringCellsByFacet(g.facet);
    case EGeometryType::None:
        break;
    }
    throw std::runtime_error("[error] getNeighboringCellsByGeometry: an undefined/None geometry has no neighboring cells.");
}

std::vector<DelaunayGraphCut::CellIndex> DelaunayGraphCut::getNeighboringCellsByFacet(const Facet& f) const
{
    std::vector<CellIndex> neighboringCells;
    neighboringCells.push_back(f.cellIndex);

    const Facet mFacet = mirrorFacet(f);
    if(!isInvalidOrInfiniteCell(mFacet.cellIndex))
        neighboringCells.push_back(mFacet.cellIndex);

    return neighboringCells;
}

std::vector<DelaunayGraphCut::CellIndex> DelaunayGraphCut::getNeighboringCellsByEdge(const Edge& e) const
{
    const std::vector<CellIndex>& v0ci = getNeighboringCellsByVertexIndex(e.v0);
    const std::vector<CellIndex>& v1ci = getNeighboringCellsByVertexIndex(e.v1);

    std::vector<CellIndex> neighboringCells;
    std::set_intersection(v0ci.begin(), v0ci.end(), v1ci.begin(), v1ci.end(), std::back_inserter(neighboringCells));
    
    return neighboringCells;
}

void DelaunayGraphCut::computeDelaunay()
{
    ALICEVISION_LOG_DEBUG("computeDelaunay GEOGRAM ...\n");

    assert(_verticesCoords.size() == _verticesAttr.size());

    long tall = clock();
    _tetrahedralization->set_vertices(_verticesCoords.size(), _verticesCoords.front().m);
    mvsUtils::printfElapsedTime(tall, "GEOGRAM Delaunay tetrahedralization ");

    initCells();

    updateVertexToCellsCache();

    ALICEVISION_LOG_DEBUG("computeDelaunay done\n");
}

void DelaunayGraphCut::initCells()
{
    _cellsAttr.resize(_tetrahedralization->nb_cells()); // or nb_finite_cells() if keeps_infinite()

    ALICEVISION_LOG_INFO(_cellsAttr.size() << " cells created by tetrahedralization.");
    for(int i = 0; i < _cellsAttr.size(); ++i)
    {
        GC_cellInfo& c = _cellsAttr[i];

        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.on = 0.0f;
        c.fullnessScore = 0.0f;
        c.emptinessScore = 0.0f;
        for(int s = 0; s < 4; ++s)
        {
            c.gEdgeVisWeight[s] = 0.0f; // weights for the 4 faces of the tetrahedron
        }
    }

    ALICEVISION_LOG_DEBUG("initCells [" << _tetrahedralization->nb_cells() << "] done");
}

void DelaunayGraphCut::displayStatistics()
{
    // Display some statistics

    StaticVector<int>* ptsCamsHist = getPtsCamsHist();
    ALICEVISION_LOG_TRACE("Histogram of number of cams per point:");
    for(int i = 0; i < ptsCamsHist->size(); ++i)
        ALICEVISION_LOG_TRACE("    " << i << ": " << mvsUtils::num2str((*ptsCamsHist)[i]));
    delete ptsCamsHist;
    /*
    StaticVector<int>* ptsNrcsHist = getPtsNrcHist();
    ALICEVISION_LOG_TRACE("Histogram of Nrc per point:");
    for(int i = 0; i < ptsNrcsHist->size(); ++i)
        ALICEVISION_LOG_TRACE("    " << i << ": " << mvsUtils::num2str((*ptsNrcsHist)[i]));
    delete ptsNrcsHist;
    */
}

StaticVector<StaticVector<int>*>* DelaunayGraphCut::createPtsCams()
{
    long t = std::clock();
    ALICEVISION_LOG_INFO("Extract visibilities.");
    int npts = getNbVertices();
    StaticVector<StaticVector<int>*>* out = new StaticVector<StaticVector<int>*>();
    out->reserve(npts);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        StaticVector<int>* cams = new StaticVector<int>();
        cams->reserve(v.getNbCameras());
        for(int c = 0; c < v.getNbCameras(); c++)
        {
            cams->push_back(v.cams[c]);
        }
        out->push_back(cams);
    } // for i

    ALICEVISION_LOG_INFO("Extract visibilities done.");

    mvsUtils::printfElapsedTime(t, "Extract visibilities ");
    return out;
}

void DelaunayGraphCut::createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams)
{
    long t = std::clock();
    ALICEVISION_LOG_INFO("Extract visibilities.");
    int npts = getNbVertices();

    out_ptsCams.reserve(npts);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        StaticVector<int> cams;
        cams.reserve(v.getNbCameras());
        for(int c = 0; c < v.getNbCameras(); c++)
        {
            cams.push_back(v.cams[c]);
        }
        out_ptsCams.push_back(cams);
    } // for i

    ALICEVISION_LOG_INFO("Extract visibilities done.");

    mvsUtils::printfElapsedTime(t, "Extract visibilities ");
}

StaticVector<int>* DelaunayGraphCut::getPtsCamsHist()
{
    int maxnCams = 0;
    for(const GC_vertexInfo& v: _verticesAttr)
    {
        maxnCams = std::max(maxnCams, (int)v.getNbCameras());
    }
    maxnCams++;
    ALICEVISION_LOG_DEBUG("maxnCams: " << maxnCams);

    StaticVector<int>* ncamsHist = new StaticVector<int>();
    ncamsHist->reserve(maxnCams);
    ncamsHist->resize_with(maxnCams, 0);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        (*ncamsHist)[v.getNbCameras()] += 1;
    }

    return ncamsHist;
}

StaticVector<int>* DelaunayGraphCut::getPtsNrcHist()
{
    int maxnnrcs = 0;
    for(const GC_vertexInfo& v: _verticesAttr)
    {
        maxnnrcs = std::max(maxnnrcs, v.nrc);
    }
    maxnnrcs++;
    ALICEVISION_LOG_DEBUG("maxnnrcs before clamp: " << maxnnrcs);
    maxnnrcs = std::min(1000, maxnnrcs);
    ALICEVISION_LOG_DEBUG("maxnnrcs: " << maxnnrcs);

    StaticVector<int>* nnrcsHist = new StaticVector<int>();
    nnrcsHist->reserve(maxnnrcs);
    nnrcsHist->resize_with(maxnnrcs, 0);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        if(v.nrc < nnrcsHist->size())
        {
            (*nnrcsHist)[v.nrc] += 1;
        }
    }

    return nnrcsHist;
}

StaticVector<int> DelaunayGraphCut::getIsUsedPerCamera() const
{
    long timer = std::clock();

    StaticVector<int> cams;
    cams.resize_with(_mp.getNbCameras(), 0);

//#pragma omp parallel for
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        const GC_vertexInfo& v = _verticesAttr[vi];
        for(int c = 0; c < v.cams.size(); ++c)
        {
            const int obsCam = v.cams[c];

//#pragma OMP_ATOMIC_WRITE
            {
                cams[obsCam] = 1;
            }
        }
    }

    mvsUtils::printfElapsedTime(timer, "getIsUsedPerCamera ");
    return cams;
}

StaticVector<int> DelaunayGraphCut::getSortedUsedCams() const
{
    const StaticVector<int> isUsed = getIsUsedPerCamera();
    StaticVector<int> out;
    out.reserve(isUsed.size());
    for(int cameraIndex = 0; cameraIndex < isUsed.size(); ++cameraIndex)
    {
        if(isUsed[cameraIndex] != 0)
            out.push_back(cameraIndex);
    }

    return out;
}

void DelaunayGraphCut::addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData)
{
  const std::size_t nbPoints = sfmData.getLandmarks().size();
  const std::size_t verticesOffset = _verticesCoords.size();

  _verticesCoords.resize(verticesOffset + nbPoints);
  _verticesAttr.resize(verticesOffset + nbPoints);

  sfmData:: Landmarks::const_iterator landmarkIt = sfmData.getLandmarks().begin();
  std::vector<Point3d>::iterator vCoordsIt = _verticesCoords.begin();
  std::vector<GC_vertexInfo>::iterator vAttrIt = _verticesAttr.begin();

  std::advance(vCoordsIt, verticesOffset);
  std::advance(vAttrIt, verticesOffset);

  std::size_t addedPoints = 0;
  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    const sfmData::Landmark& landmark = landmarkIt->second;
    const Point3d p(landmark.X(0), landmark.X(1), landmark.X(2));

    if(mvsUtils::isPointInHexahedron(p, hexah))
    {
      *vCoordsIt = p;

      vAttrIt->nrc = landmark.observations.size();
      vAttrIt->cams.reserve(vAttrIt->nrc);

      for(const auto& observationPair : landmark.observations)
        vAttrIt->cams.push_back(_mp.getIndexFromViewId(observationPair.first));

      vAttrIt->pixSize = _mp.getCamsMinPixelSize(p, vAttrIt->cams);

      ++vCoordsIt;
      ++vAttrIt;
      ++addedPoints;
    }
    ++landmarkIt;
  }
  if(addedPoints != nbPoints)
  {
    _verticesCoords.resize(verticesOffset + addedPoints);
    _verticesAttr.resize(verticesOffset + addedPoints);
  }
  ALICEVISION_LOG_WARNING("Add " << addedPoints << " new points for the SfM.");
}

void DelaunayGraphCut::addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist)
{
    int addedPoints = 0;
    for(int camid = 0; camid < cams.size(); camid++)
    {
        int rc = cams[camid];
        {
            const Point3d p(_mp.CArr[rc].x, _mp.CArr[rc].y, _mp.CArr[rc].z);
            const GEO::index_t vi = locateNearestVertex(p);

            if((vi == GEO::NO_VERTEX) || ((_verticesCoords[vi] - _mp.CArr[rc]).size() > minDist))
            {
                const GEO::index_t nvi = _verticesCoords.size();
                _verticesCoords.push_back(p);

                GC_vertexInfo newv;
                newv.nrc = 0;

                _camsVertexes[rc] = nvi;

                _verticesAttr.push_back(newv);
                ++addedPoints;
            }
            else
            {
                _camsVertexes[rc] = vi;
            }
        }
    }
    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new points for the " << cams.size() << " cameras centers.");
}

void DelaunayGraphCut::addPointsToPreventSingularities(const Point3d voxel[8], float minDist)
{
    Point3d vcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0f;
    Point3d extrPts[6];
    Point3d fcg;
    fcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3]) / 4.0f;
    extrPts[0] = fcg + (fcg - vcg) / 10.0f;
    fcg = (voxel[0] + voxel[4] + voxel[7] + voxel[3]) / 4.0f;
    extrPts[1] = fcg + (fcg - vcg) / 10.0f;
    fcg = (voxel[0] + voxel[1] + voxel[5] + voxel[4]) / 4.0f;
    extrPts[2] = fcg + (fcg - vcg) / 10.0f;
    fcg = (voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 4.0f;
    extrPts[3] = fcg + (fcg - vcg) / 10.0f;
    fcg = (voxel[1] + voxel[5] + voxel[6] + voxel[2]) / 4.0f;
    extrPts[4] = fcg + (fcg - vcg) / 10.0f;
    fcg = (voxel[3] + voxel[2] + voxel[6] + voxel[7]) / 4.0f;
    extrPts[5] = fcg + (fcg - vcg) / 10.0f;
    int addedPoints = 0;
    for(int i = 0; i < 6; i++)
    {
        const Point3d p(extrPts[i].x, extrPts[i].y, extrPts[i].z);
        const GEO::index_t vi = locateNearestVertex(p);

        if((vi == GEO::NO_VERTEX) || ((_verticesCoords[vi] - extrPts[i]).size() > minDist))
        {
            _verticesCoords.push_back(p);
            GC_vertexInfo newv;
            newv.nrc = 0;

            _verticesAttr.push_back(newv);
            ++addedPoints;
        }
    }
    ALICEVISION_LOG_WARNING("Add " << addedPoints << " points to prevent singularities");
}

void DelaunayGraphCut::densifyWithHelperPoints(int nbFront, int nbBack, double scale)
{
    if(nbFront <= 0 && nbBack <= 0)
        return;

    const std::size_t nbInputVertices = _verticesCoords.size();

    std::vector<Point3d> newHelperPoints;
    newHelperPoints.reserve((nbFront + nbBack) * nbInputVertices);

    for(std::size_t vi = 0; vi < nbInputVertices; ++vi)
    {
        const Point3d& v = _verticesCoords[vi];
        const GC_vertexInfo& vAttr = _verticesAttr[vi];

        if(vAttr.cams.empty() || vAttr.pixSize <= std::numeric_limits<float>::epsilon())
            continue;

        Point3d mainCamDir;
        for(int camId: vAttr.cams)
        {
            const Point3d& cam = _mp.CArr[camId];
            const Point3d d = (cam - v).normalize();
            mainCamDir += d;
        }
        mainCamDir /= double(vAttr.cams.size());
        mainCamDir = mainCamDir.normalize() * vAttr.pixSize;
        
        for(int iFront = 1; iFront < nbFront + 1; ++iFront)
            newHelperPoints.push_back(v + mainCamDir * iFront * scale);
        for(int iBack = 1; iBack < nbBack + 1; ++iBack)
            newHelperPoints.push_back(v - mainCamDir * iBack * scale);
    }
    _verticesCoords.resize(nbInputVertices + newHelperPoints.size());
    _verticesAttr.resize(nbInputVertices + newHelperPoints.size());
    for(std::size_t vi = 0; vi < newHelperPoints.size(); ++vi)
    {
        _verticesCoords[nbInputVertices + vi] = newHelperPoints[vi];
        // GC_vertexInfo& vAttr = _verticesAttr[nbInputVertices + vi];
        // Keep vertexInfo with default/empty values, so they will be removed at the end as other helper points
        // vAttr.nrc = 0;
    }

    ALICEVISION_LOG_WARNING("Densify the " << nbInputVertices << " vertices with " << newHelperPoints.size()
                                           << " new helper points.");
}

void DelaunayGraphCut::addGridHelperPoints(int helperPointsGridSize, const Point3d voxel[8], float minDist)
{
    if(helperPointsGridSize <= 0)
        return;

    int ns = helperPointsGridSize;
    float md = 1.0f / 500.0f;
    Point3d vx = (voxel[1] - voxel[0]);
    Point3d vy = (voxel[3] - voxel[0]);
    Point3d vz = (voxel[4] - voxel[0]);
    Point3d O = voxel[0] + vx * md + vy * md + vz * md;
    vx = vx - vx * 2.0f * md;
    vy = vy - vy * 2.0f * md;
    vz = vz - vz * 2.0f * md;

    float maxSize = 2.0f * (O - voxel[0]).size();
    Point3d CG = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0f;
    
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    std::mt19937 generator(seed != 0 ? seed : std::random_device{}());
    auto rand = std::bind(std::uniform_real_distribution<float>{0.0, 1.0}, generator);

    int addedPoints = 0;
    for(int x = 0; x <= ns; ++x)
    {
        for(int y = 0; y <= ns; ++y)
        {
            for(int z = 0; z <= ns; ++z)
            {
                Point3d pt = voxel[0] + vx * ((float)x / (float)ns) + vy * ((float)y / (float)ns) +
                             vz * ((float)z / (float)ns);
                pt = pt + (CG - pt).normalize() * (maxSize * rand());

                const Point3d p(pt.x, pt.y, pt.z);
                const GEO::index_t vi = locateNearestVertex(p);

                // if there is no nearest vertex or the nearest vertex is not too close
                if((vi == GEO::NO_VERTEX) || ((_verticesCoords[vi] - pt).size() > minDist))
                {
                    _verticesCoords.push_back(p);
                    GC_vertexInfo newv;
                    newv.nrc = 0;

                    _verticesAttr.push_back(newv);
                    ++addedPoints;
                }
            }
        }
    }

    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new helper points for a 3D grid of " << ns << "x" << ns << "x" << ns <<".");
}

void DelaunayGraphCut::addMaskHelperPoints(const Point3d voxel[8], const StaticVector<int>& cams, const FuseParams& params)
{
    if(params.maskHelperPointsWeight <= 0.0)
        return;

    ALICEVISION_LOG_INFO("Add Mask Helper Points.");

    Point3d inflatedVoxel[8];
    mvsUtils::inflateHexahedron(voxel, inflatedVoxel, 1.01f);

    std::size_t nbPixels = 0;
    for(const auto& imgParams : _mp.getImagesParams())
    {
        nbPixels += imgParams.size;
    }
    // int step = std::floor(std::sqrt(double(nbPixels) / double(params.maxInputPoints)));
    // step = std::max(step, params.minStep);
    const int step = 1;
    int nbAddedPoints = 0;

    ALICEVISION_LOG_INFO("Load depth maps and add points.");
    {
        for(int c = 0; c < cams.size(); c++)
        {
            std::vector<float> depthMap;
            int width, height;
            {
                const std::string depthMapFilepath = getFileNameFromIndex(_mp, c, mvsUtils::EFileType::depthMap, 0);
                imageIO::readImage(depthMapFilepath, width, height, depthMap, imageIO::EImageColorSpace::NO_CONVERSION);
                if(depthMap.empty())
                {
                    ALICEVISION_LOG_WARNING("Empty depth map: " << depthMapFilepath);
                    continue;
                }
            }

            int syMax = std::ceil(height / step);
            int sxMax = std::ceil(width / step);

            for(int sy = 0; sy < syMax; ++sy)
            {
                for(int sx = 0; sx < sxMax; ++sx)
                {
                    float bestScore = 0;

                    int bestX = 0;
                    int bestY = 0;
                    for(int y = sy * step, ymax = std::min((sy + 1) * step, height); y < ymax; ++y)
                    {
                        for(int x = sx * step, xmax = std::min((sx + 1) * step, width); x < xmax; ++x)
                        {
                            const std::size_t index = y * width + x;
                            const float depth = depthMap[index];
                            
                            // -2 means that the pixels should be masked-out with mask helper points
                            if(depth > -1.5f)
                                continue;

                            int nbValidDepth = 0;
                            const int kernelSize = params.maskBorderSize;
                            for(int ly = std::max(y - kernelSize, 0), lyMax = std::min(y + kernelSize, height - 1);
                                ly < lyMax; ++ly)
                            {
                                for(int lx = std::max(x - kernelSize, 0), lxMax = std::min(x + kernelSize, width - 1);
                                    lx < lxMax; ++lx)
                                {
                                    if(depthMap[ly * width + lx] > 0.0f)
                                        ++nbValidDepth;
                                }
                            }

                            const float score = nbValidDepth; // TODO: best score based on nbValidDepth and kernel size ?
                            if(score > bestScore)
                            {
                                bestScore = score;
                                bestX = x;
                                bestY = y;
                            }
                        }
                    }
                    if(bestScore > 0.0f)
                    {
                        const Point3d& cam = _mp.CArr[c];
                        Point3d maxP = cam + (_mp.iCamArr[c] * Point2d((float)bestX, (float)bestY)).normalize() * 10000000.0; 
                        StaticVector<Point3d>* intersectionsPtr = mvsUtils::lineSegmentHexahedronIntersection(cam, maxP, inflatedVoxel);

                        if(intersectionsPtr->size() <= 0)
                            continue;

                        Point3d p;
                        double maxDepth = std::numeric_limits<double>::min();
                        for(Point3d& i : *intersectionsPtr)
                        {
                            const double depth = dist(cam, i);
                            if(depth > maxDepth)
                            {
                                p = i;
                                maxDepth = depth;
                            }
                        }
                        GC_vertexInfo newv;
                        newv.nrc = params.maskHelperPointsWeight;
                        newv.pixSize = 0.0f;
                        newv.cams.push_back_distinct(c);

                        _verticesAttr.push_back(newv);
                        _verticesCoords.emplace_back(p);
                        ++nbAddedPoints;
                    }
                }
            }
        }
    }
    ALICEVISION_LOG_INFO("Added Points: " << nbAddedPoints);
    ALICEVISION_LOG_INFO("Add Mask Helper Points done.");
}

void DelaunayGraphCut::fuseFromDepthMaps(const StaticVector<int>& cams, const Point3d voxel[8], const FuseParams& params)
{
    ALICEVISION_LOG_INFO("fuseFromDepthMaps, maxVertices: " << params.maxPoints);

    // Load depth from depth maps, select points per depth maps (1 value per tile).
    // Filter points inside other points (with a volume defined by the pixelSize)
    // If too much points at the end, increment a coefficient factor on the pixel size
    // and iterate to fuse points until we get the right amount of points.

    // unsigned long nbValidDepths = computeNumberOfAllPoints(mp, 0);
    // int stepPts = std::ceil((double)nbValidDepths / (double)maxPoints);
    std::size_t nbPixels = 0;
    for(const auto& imgParams: _mp.getImagesParams())
    {
        nbPixels += imgParams.size;
    }
    int step = std::floor(std::sqrt(double(nbPixels) / double(params.maxInputPoints)));
    step = std::max(step, params.minStep);
    std::size_t realMaxVertices = 0;
    std::vector<int> startIndex(_mp.getNbCameras(), 0);
    for(int i = 0; i < _mp.getNbCameras(); ++i)
    {
        const auto& imgParams = _mp.getImageParams(i);
        startIndex[i] = realMaxVertices;
        realMaxVertices += std::ceil(imgParams.width / step) * std::ceil(imgParams.height / step);
    }
    std::vector<Point3d> verticesCoordsPrepare(realMaxVertices);
    std::vector<double> pixSizePrepare(realMaxVertices);
    std::vector<float> simScorePrepare(realMaxVertices);

    // counter for points filtered based on the number of observations (minVis)
    int minVisCounter = 0;

    ALICEVISION_LOG_INFO("simFactor: " << params.simFactor);
    ALICEVISION_LOG_INFO("nbPixels: " << nbPixels);
    ALICEVISION_LOG_INFO("maxVertices: " << params.maxPoints);
    ALICEVISION_LOG_INFO("step: " << step);
    ALICEVISION_LOG_INFO("realMaxVertices: " << realMaxVertices);
    ALICEVISION_LOG_INFO("minVis: " << params.minVis);

    ALICEVISION_LOG_INFO("Load depth maps and add points.");
    {
        omp_set_nested(1);
        #pragma omp parallel for num_threads(3)
        for(int c = 0; c < cams.size(); c++)
        {
            std::vector<float> depthMap;
            std::vector<float> simMap;
            std::vector<unsigned char> numOfModalsMap;
            int width, height;
            {
                const std::string depthMapFilepath = getFileNameFromIndex(_mp, c, mvsUtils::EFileType::depthMap, 0);
                imageIO::readImage(depthMapFilepath, width, height, depthMap, imageIO::EImageColorSpace::NO_CONVERSION);
                if(depthMap.empty())
                {
                    ALICEVISION_LOG_WARNING("Empty depth map: " << depthMapFilepath);
                    continue;
                }
                int wTmp, hTmp;
                const std::string simMapFilepath = getFileNameFromIndex(_mp, c, mvsUtils::EFileType::simMap, 0);
                // If we have a simMap in input use it,
                // else init with a constant value.
                if(boost::filesystem::exists(simMapFilepath))
                {
                    imageIO::readImage(simMapFilepath, wTmp, hTmp, simMap, imageIO::EImageColorSpace::NO_CONVERSION);
                    if(wTmp != width || hTmp != height)
                        throw std::runtime_error("Wrong sim map dimensions: " + simMapFilepath);
                    {
                        std::vector<float> simMapTmp(simMap.size());
                        imageAlgo::convolveImage(width, height, simMap, simMapTmp, "gaussian",
                    params.simGaussianSizeInit, params.simGaussianSizeInit); simMap.swap(simMapTmp);
                    }
                }
                else
                {
                    ALICEVISION_LOG_WARNING("simMap file can't be found.");
                    simMap.resize(width * height, -1);
                }

                const std::string nmodMapFilepath = getFileNameFromIndex(_mp, c, mvsUtils::EFileType::nmodMap, 0);
                // If we have an nModMap in input (from depthmapfilter) use it,
                // else init with a constant value.
                if(boost::filesystem::exists(nmodMapFilepath))
                {
                    imageIO::readImage(nmodMapFilepath, wTmp, hTmp, numOfModalsMap,
                                       imageIO::EImageColorSpace::NO_CONVERSION);
                    if(wTmp != width || hTmp != height)
                        throw std::runtime_error("Wrong nmod map dimensions: " + nmodMapFilepath);
                }
                else
                {
                    ALICEVISION_LOG_WARNING("nModMap file can't be found.");
                    numOfModalsMap.resize(width*height, 1);
                }
            }

            int syMax = std::ceil(height/step);
            int sxMax = std::ceil(width/step);
            #pragma omp parallel for
            for(int sy = 0; sy < syMax; ++sy)
            {
                for(int sx = 0; sx < sxMax; ++sx)
                {
                    int index = startIndex[c] + sy * sxMax + sx;
                    float bestDepth = std::numeric_limits<float>::max();
                    float bestScore = 0;
                    float bestSimScore = 0;
                    int bestX = 0;
                    int bestY = 0;
                    for(int y = sy * step, ymax = std::min((sy+1) * step, height);
                        y < ymax; ++y)
                    {
                        for(int x = sx * step, xmax = std::min((sx+1) * step, width);
                            x < xmax; ++x)
                        {
                            const std::size_t index = y * width + x;
                            const float depth = depthMap[index];
                            if(depth <= 0.0f)
                                continue;

                            int numOfModals = 0;
                            const int scoreKernelSize = 1;
                            for(int ly = std::max(y-scoreKernelSize, 0), lyMax = std::min(y+scoreKernelSize, height-1); ly < lyMax; ++ly)
                            {
                                for(int lx = std::max(x-scoreKernelSize, 0), lxMax = std::min(x+scoreKernelSize, width-1); lx < lxMax; ++lx)
                                {
                                    if(depthMap[ly * width + lx] > 0.0f)
                                    {
                                        numOfModals += 10 + int(numOfModalsMap[ly * width + lx]);
                                    }
                                }
                            }
                            float sim = simMap[index];
                            sim = sim < 0.0f ?  0.0f : sim; // clamp values < 0
                            // remap similarity values from [-1;+1] to [+1;+simScale]
                            // interpretation is [goodSimilarity;badSimilarity]
                            const float simScore = 1.0f + sim * params.simFactor;

                            const float score = numOfModals + (1.0f / simScore);
                            if(score > bestScore)
                            {
                                bestDepth = depth;
                                bestScore = score;
                                bestSimScore = simScore;
                                bestX = x;
                                bestY = y;
                            }
                        }
                    }
                    if(bestScore < 3*13)
                    {
                        // discard the point
                        pixSizePrepare[index] = -1.0;
                    }
                    else
                    {
                        Point3d p = _mp.CArr[c] + (_mp.iCamArr[c] * Point2d((float)bestX, (float)bestY)).normalize() * bestDepth;
                        
                        // TODO: isPointInHexahedron: here or in the previous loop per pixel to not loose point?
                        if(voxel == nullptr || mvsUtils::isPointInHexahedron(p, voxel)) 
                        {
                            verticesCoordsPrepare[index] = p;
                            simScorePrepare[index] = bestSimScore;
                            pixSizePrepare[index] = _mp.getCamPixelSize(p, c);
                        }
                        else
                        {
                            // discard the point
                            // verticesCoordsPrepare[index] = p;
                            pixSizePrepare[index] = -1.0;
                        }
                    }
                }
            }
        }
        omp_set_nested(0);
    }

    ALICEVISION_LOG_INFO("Filter initial 3D points by pixel size to remove duplicates.");

    filterByPixSize(verticesCoordsPrepare, pixSizePrepare, params.pixSizeMarginInitCoef, simScorePrepare);
    // remove points if pixSize == -1
    removeInvalidPoints(verticesCoordsPrepare, pixSizePrepare, simScorePrepare);

    ALICEVISION_LOG_INFO("3D points loaded and filtered to " << verticesCoordsPrepare.size() << " points.");

    ALICEVISION_LOG_INFO("Init visibilities to compute angle scores");
    std::vector<GC_vertexInfo> verticesAttrPrepare(verticesCoordsPrepare.size());

    // Compute the vertices positions and simScore from all input depthMap/simMap images,
    // and declare the visibility information (the cameras indexes seeing the vertex).
    createVerticesWithVisibilities(cams, verticesCoordsPrepare, pixSizePrepare, simScorePrepare,
                                   verticesAttrPrepare, _mp, params.simFactor, params.voteMarginFactor, params.contributeMarginFactor, params.simGaussianSize);

    ALICEVISION_LOG_INFO("Compute max angle per point");

    ALICEVISION_LOG_INFO("angleFactor: " << params.angleFactor);
    // Compute max visibility angle per point
    // and weight simScore with angular score
#if defined(FUSE_COMPUTE_ANGLE_STATS) && !defined(OMP_HAVE_MIN_MAX_REDUCTION)
    ALICEVISION_LOG_DEBUG("Disable angle stats computation: OpenMP does not provide required min/max reduction clauses.");
#undef FUSE_COMPUTE_ANGLE_STATS
#endif

#ifdef FUSE_COMPUTE_ANGLE_STATS
    double stat_minAngle = std::numeric_limits<double>::max(), stat_maxAngle = 0.0;
    double stat_minAngleScore = std::numeric_limits<double>::max(), stat_maxAngleScore = 0.0;
    #pragma omp parallel for reduction(max: stat_maxAngle,stat_maxAngleScore) reduction(min: stat_minAngle,stat_minAngleScore)
#else
#pragma omp parallel for
#endif
    for(int vIndex = 0; vIndex < verticesCoordsPrepare.size(); ++vIndex)
    {
        if(pixSizePrepare[vIndex] == -1.0)
        {
            continue;
        }
        const std::vector<int>& visCams = verticesAttrPrepare[vIndex].cams.getData();
        if(visCams.empty())
        {
            ALICEVISION_LOG_WARNING("BAD: visCams.empty()");
        }
        double maxAngle = 0.0;
        for(int i: visCams)
        {
            for(int j: visCams)
            {
                if(i == j)
                    continue;
                double angle = angleBetwABandAC(verticesCoordsPrepare[vIndex], _mp.CArr[i], _mp.CArr[j]);
                maxAngle = std::max(angle, maxAngle);
            }
        }
        // Kill the point if the angle is too small
        if(maxAngle < params.minAngleThreshold)
        {
            pixSizePrepare[vIndex] = -1;
            continue;
        }
        // Filter points based on their number of observations
        if(visCams.size() < params.minVis)
        {
            pixSizePrepare[vIndex] = -1;
            minVisCounter += 1;
            continue;
        }

        const double angleScore = 1.0 + params.angleFactor / maxAngle;
        // Combine angleScore with simScore
        simScorePrepare[vIndex] = simScorePrepare[vIndex] * angleScore;

#ifdef FUSE_COMPUTE_ANGLE_STATS
        stat_minAngle = std::min(stat_minAngle, maxAngle);
        stat_maxAngle = std::max(stat_maxAngle, maxAngle);

        stat_minAngleScore = std::min(stat_minAngleScore, angleScore);
        stat_maxAngleScore = std::max(stat_maxAngleScore, angleScore);
#endif
    }
#ifdef FUSE_COMPUTE_ANGLE_STATS
    ALICEVISION_LOG_INFO("Angle min: " << stat_minAngle << ", max: " << stat_maxAngle << ".");
    ALICEVISION_LOG_INFO("Angle score min: " << stat_minAngleScore << ", max: " << stat_maxAngleScore << ".");
#endif
    ALICEVISION_LOG_INFO((minVisCounter) << " points filtered based on the number of observations (minVis). ");
    removeInvalidPoints(verticesCoordsPrepare, pixSizePrepare, simScorePrepare, verticesAttrPrepare);

    ALICEVISION_LOG_INFO("Filter by angle score and sim score");

    // while more points than the max points (with a limit to 20 iterations).
    double pixSizeMarginFinalCoef = params.pixSizeMarginFinalCoef;
    for(int filteringIt = 0; filteringIt < 20; ++filteringIt)
    {
        // Filter points with new simScore
        filterByPixSize(verticesCoordsPrepare, pixSizePrepare, pixSizeMarginFinalCoef, simScorePrepare);
        removeInvalidPoints(verticesCoordsPrepare, pixSizePrepare, simScorePrepare, verticesAttrPrepare);

        if(verticesCoordsPrepare.size() < params.maxPoints)
        {
            ALICEVISION_LOG_INFO("The number of points is below the max number of vertices.");
            break;
        }
        else
        {
            pixSizeMarginFinalCoef *= 1.5;
            ALICEVISION_LOG_INFO("Increase pixel size margin coef to " << pixSizeMarginFinalCoef << ", nb points: " << verticesCoordsPrepare.size() << ", maxVertices: " << params.maxPoints);
        }
    }
    ALICEVISION_LOG_INFO("3D points loaded and filtered to " << verticesCoordsPrepare.size() << " points (maxVertices is " << params.maxPoints << ").");

    if(params.refineFuse)
    {
        ALICEVISION_LOG_INFO("Create final visibilities");
        // Initialize the vertice attributes and declare the visibility information
        createVerticesWithVisibilities(cams, verticesCoordsPrepare, pixSizePrepare, simScorePrepare,
                                       verticesAttrPrepare, _mp, params.simFactor, params.voteMarginFactor, params.contributeMarginFactor, params.simGaussianSize);
    }

    if(verticesCoordsPrepare.empty())
        throw std::runtime_error("Depth map fusion gives an empty result.");

    ALICEVISION_LOG_WARNING("fuseFromDepthMaps done: " << verticesCoordsPrepare.size() << " points created.");

    // Insert the new elements
    if(_verticesCoords.empty())
    {
        // replace with the new points if emoty
        _verticesCoords.swap(verticesCoordsPrepare);
        _verticesAttr.swap(verticesAttrPrepare);
    }
    else
    {
        // concatenate the new elements with the previous ones
        _verticesCoords.insert(_verticesCoords.end(), verticesCoordsPrepare.begin(), verticesCoordsPrepare.end() );
        _verticesAttr.insert(_verticesAttr.end(), verticesAttrPrepare.begin(), verticesAttrPrepare.end() );
    }
}

void DelaunayGraphCut::computeVerticesSegSize(std::vector<GC_Seg>& out_segments, const std::vector<bool>& useVertex, float alpha) // allPoints=true, alpha=0
{
    ALICEVISION_LOG_DEBUG("DelaunayGraphCut::computeVerticesSegSize");
    out_segments.clear();

    int scalePS = _mp.userParams.get<int>("global.scalePS", 1);
    int step = _mp.userParams.get<int>("global.step", 1);
    float pointToJoinPixSizeDist = (float)_mp.userParams.get<double>("delaunaycut.pointToJoinPixSizeDist", 2.0) *
                                   (float)scalePS * (float)step * 2.0f;

    std::vector<Pixel> edges;
    edges.reserve(_verticesAttr.size());

    if(alpha < 1.0f)
    {
        alpha = 2.0f * std::max(2.0f, pointToJoinPixSizeDist);
    }

    // long t1 = mvsUtils::initEstimate();
    assert(_verticesCoords.size() == _verticesAttr.size());

    for(VertexIndex vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        const GC_vertexInfo& v = _verticesAttr[vi];
        const Point3d& p = _verticesCoords[vi];
        if((v.getNbCameras() > 0) && (useVertex.empty() || useVertex[vi]))
        {
            int rc = v.getCamera(0);

            // go through all the neighbouring points
            GEO::vector<VertexIndex> adjVertices;
            _tetrahedralization->get_neighbors(vi, adjVertices);

            for(VertexIndex nvi: adjVertices)
            {
                const GC_vertexInfo& nv = _verticesAttr[nvi];
                const Point3d& np = _verticesCoords[nvi];
                if((vi != nvi) && (useVertex.empty() || useVertex[nvi]))
                {
                    if((p - np).size() <
                       alpha * _mp.getCamPixelSize(p, rc)) // TODO FACA: why do we fuse again? And only based on the pixSize of the first camera??
                    {
                        if(vi < nvi) // to remove duplicates
                        {
                            edges.emplace_back(vi, nvi);
                        }
                    }
                }
            }
        }
        // mvsUtils::printfEstimate(vi, _verticesAttr.size(), t1);
    }
    // mvsUtils::finishEstimate();

    Universe u(_verticesAttr.size());

    // t1 = mvsUtils::initEstimate();
    int s = (int)edges.size(); // Fuse all edges collected to be merged
    for(int i = 0; i < s; i++)
    {
        int a = u.find(edges[i].x);
        int b = u.find(edges[i].y);
        if(a != b) // TODO FACA: Are they not different in all cases?
        {
            u.join(a, b);
        }
        // mvsUtils::printfEstimate(i, s, t1);
    }
    // mvsUtils::finishEstimate();

    out_segments.resize(_verticesAttr.size());

    // Last loop over vertices to update segId
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        GC_vertexInfo& v = _verticesAttr[vi];
        if(v.isVirtual())
            continue;

        GC_Seg& s = out_segments[vi];
        int a = u.find(vi);
        s.segSize = u.elts[a].size;
        s.segId = a;
    }

    ALICEVISION_LOG_DEBUG("DelaunayGraphCut::computeVerticesSegSize done.");
}

void DelaunayGraphCut::removeSmallSegs(const std::vector<GC_Seg>& segments, int minSegSize)
{
    ALICEVISION_LOG_DEBUG("removeSmallSegs: " << minSegSize);
    StaticVector<int> toRemove;
    toRemove.reserve(getNbVertices());

    for(int i = 0; i < _verticesAttr.size(); ++i)
    {
        const GC_vertexInfo& v = _verticesAttr[i];
        const GC_Seg& s = segments[i];
        if(v.isReal() && s.segSize < minSegSize)
        {
            toRemove.push_back(i);
        }
    }
    ALICEVISION_LOG_WARNING("removeSmallSegs removing " << toRemove.size() << " cells.");

    for(int i = 0; i < toRemove.size(); i++)
    {
        int iR = toRemove[i];
        GC_vertexInfo& v = _verticesAttr[iR];

        v.cams.clear();
        // T.remove(fit); // TODO GEOGRAM
    }

}

DelaunayGraphCut::GeometryIntersection
DelaunayGraphCut::intersectNextGeom(const DelaunayGraphCut::GeometryIntersection& inGeometry,
    const Point3d& originPt,
    const Point3d& dirVect, Point3d& intersectPt,
    const double epsilonFactor, const Point3d& lastIntersectPt) const
{
    GeometryIntersection bestMatch;
    Point3d bestMatchIntersectPt;

    switch (inGeometry.type)
    {
    case EGeometryType::Facet:
    {
        const CellIndex tetrahedronIndex = inGeometry.facet.cellIndex;

        // Test all facets of the tetrahedron using i as localVertexIndex to define next intersectionFacet
        for (int i = 0; i < 4; ++i)
        {
            // Because we can't intersect with incoming facet (same localVertexIndex)
            if (i == inGeometry.facet.localVertexIndex)
                continue;

            const Facet intersectionFacet(tetrahedronIndex, i);
            bool ambiguous = false;

            const GeometryIntersection result = rayIntersectTriangle(originPt, dirVect, intersectionFacet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);
            if (result.type != EGeometryType::None)
            {
                if (!ambiguous)
                    return result;

                // Retrieve the best intersected point (farthest from origin point)
                if (bestMatch.type == EGeometryType::None || (originPt - intersectPt).size() > (originPt - bestMatchIntersectPt).size())
                {
                    bestMatchIntersectPt = intersectPt;
                    bestMatch = result;
                }
            }
        }
    }
    break;

    case EGeometryType::Vertex:
    {
        for (CellIndex adjCellIndex : getNeighboringCellsByVertexIndex(inGeometry.vertexIndex))
        {
            if(isInvalidOrInfiniteCell(adjCellIndex))
                continue;

            // Get local vertex index
            const VertexIndex localVertexIndex = _tetrahedralization->index(adjCellIndex, inGeometry.vertexIndex);

            // Define the facet to intersect
            const Facet facet(adjCellIndex, localVertexIndex);
            bool ambiguous = false;

            const GeometryIntersection result = rayIntersectTriangle(originPt, dirVect, facet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);
            if (result.type != EGeometryType::None)
            {
                if (!ambiguous)
                    return result;

                // Retrieve the best intersected point (farthest from origin point)
                if (bestMatch.type == EGeometryType::None || (originPt - intersectPt).size() > (originPt - bestMatchIntersectPt).size())
                {
                    bestMatchIntersectPt = intersectPt;
                    bestMatch = result;
                }
            }
        }
    }
    break;

    case EGeometryType::Edge:
    {
        GeometryIntersection result;

        for (CellIndex adjCellIndex : getNeighboringCellsByEdge(inGeometry.edge))
        {
            if(isInvalidOrInfiniteCell(adjCellIndex))
                continue;
            // Local vertices indices
            const VertexIndex lvi0 = _tetrahedralization->index(adjCellIndex, inGeometry.edge.v0);
            const VertexIndex lvi1 = _tetrahedralization->index(adjCellIndex, inGeometry.edge.v1);

            // The two facets that do not touch this edge need to be tested
            const std::array<Facet, 2> opositeFacets{ {{adjCellIndex, lvi0}, {adjCellIndex, lvi1}} };

            for (const Facet& facet : opositeFacets)
            {
                bool ambiguous = false;
                const GeometryIntersection result = rayIntersectTriangle(originPt, dirVect, facet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);

                if (result.type == EGeometryType::Edge)
                {
                    if(result.edge.isSameUndirectionalEdge(inGeometry.edge))
                    {
                        ALICEVISION_LOG_ERROR("[intersectNextGeom] intersect the input edge itself, ignore intersection: inGeometry: " << inGeometry << ", intersected geo: " << result);
                        // do not intersect the input edge itself
                        continue;
                    }
                    // ALICEVISION_LOG_WARNING("[intersectNextGeom] intersect a new edge from an edge: inGeometry: " << inGeometry << ", intersected geo: " << result << ", intersectPt: " << intersectPt << ", lastIntersectPt: " << lastIntersectPt);
                }
                if (result.type != EGeometryType::None)
                {
                    if (!ambiguous)
                        return result;

                    // Retrieve the best intersected point (farthest from origin point)
                    if (bestMatch.type == EGeometryType::None || (originPt - intersectPt).size() > (originPt - bestMatchIntersectPt).size())
                    {
                        bestMatchIntersectPt = intersectPt;
                        bestMatch = result;
                    }
                }
            }
        }
    }
    break;

    case EGeometryType::None:
        //throw std::runtime_error("[intersectNextGeom] intersection with input none geometry should not happen.");
        ALICEVISION_LOG_WARNING("[intersectNextGeom] intersection with input none geometry should not happen: " << inGeometry);
        break;
    }

    intersectPt = bestMatchIntersectPt;
    return bestMatch;
}

DelaunayGraphCut::GeometryIntersection DelaunayGraphCut::rayIntersectTriangle(const Point3d& originPt,
    const Point3d& DirVec,
    const DelaunayGraphCut::Facet& facet,
    Point3d& intersectPt,
    const double epsilonFactor, bool &ambiguous, const Point3d* lastIntersectPt) const
{
    ambiguous = false;

    const VertexIndex AvertexIndex = getVertexIndex(facet, 0);
    const VertexIndex BvertexIndex = getVertexIndex(facet, 1);
    const VertexIndex CvertexIndex = getVertexIndex(facet, 2);

    const Point3d* A = &_verticesCoords[AvertexIndex];
    const Point3d* B = &_verticesCoords[BvertexIndex];
    const Point3d* C = &_verticesCoords[CvertexIndex];

    const double ABSize = (*A - *B).size();
    const double BCSize = (*B - *C).size();
    const double ACSize = (*A - *C).size();

    const double marginEpsilon = std::min(std::min(ABSize, BCSize), ACSize) * epsilonFactor;
    const double ambiguityEpsilon = (ABSize + BCSize + ACSize) / 3.0 * 1.0e-2;

    Point3d tempIntersectPt;
    const Point2d triangleUv = getLineTriangleIntersectBarycCoords(&tempIntersectPt, A, B, C, &originPt, &DirVec);

    if (!isnormal(tempIntersectPt.x) || !isnormal(tempIntersectPt.y) || !isnormal(tempIntersectPt.z))
    {
        // This is not suppose to happen in real life, we log a warning instead of raising an exeption if we face a border case
        // ALICEVISION_LOG_WARNING("Invalid/notNormal intersection point found during rayIntersectTriangle.");
        return GeometryIntersection();
    }

    const double u = triangleUv.x; // A to C
    const double v = triangleUv.y; // A to B

    // If we find invalid uv coordinate
    if (!std::isfinite(u) || !std::isfinite(v))
        return GeometryIntersection();

    // Ouside the triangle with marginEpsilon margin
    if (u < -marginEpsilon || v < -marginEpsilon || (u + v) > (1.0 + marginEpsilon))
        return GeometryIntersection();

    // In case intersectPt is provided, check if intersectPt is in front of lastIntersectionPt 
    // in the DirVec direction to ensure that we are moving forward in the right direction
    if (lastIntersectPt != nullptr)
    {
        const Point3d diff = tempIntersectPt - *lastIntersectPt;
        const double dotValue = dot(DirVec, diff.normalize());
        if(dotValue < marginEpsilon || diff.size() < 100 * std::numeric_limits<double>::min())
        {
            return GeometryIntersection();
        }

        if (diff.size() < ambiguityEpsilon)
        {
            ambiguous = true;
        }
    }

    // Change intersection point only if tempIntersectionPt is in the right direction (mean we intersect something)
    intersectPt = tempIntersectPt;

    if (v < marginEpsilon) // along A C edge
    {
        if (u < marginEpsilon)
        {
            intersectPt = *A;
            return GeometryIntersection(AvertexIndex); // vertex A
        }
        if (u > 1.0 - marginEpsilon)
        {
            intersectPt = *C;
            return GeometryIntersection(CvertexIndex); // vertex C
        }

        return GeometryIntersection(Edge(AvertexIndex, CvertexIndex)); // edge AC
    }

    if (u < marginEpsilon) // along A B edge
    {
        if (v > 1.0 - marginEpsilon)
        {
            intersectPt = *B;
            return GeometryIntersection(BvertexIndex); // vertex B
        }

        return GeometryIntersection(Edge(AvertexIndex, BvertexIndex)); // edge AB
    }

    if (u + v > 1.0 - marginEpsilon)
        return GeometryIntersection(Edge(BvertexIndex, CvertexIndex)); // edge BC

    return GeometryIntersection(facet);
}

float DelaunayGraphCut::distFcn(float maxDist, float dist, float distFcnHeight) const
{
    // distFcnHeight ... 0 for distFcn == 1 for all dist, 0.1 distFcn std::min
    // 0.9, ...., 1.0 dist
    // fcn std::min 0.0f ... check matlab
    // MATLAB: distFcnHeight=0.5; maxDist = 10; dist = 0:0.1:20;
    // plot(dist,1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist)));
    return 1.0f - distFcnHeight * std::exp(-(dist * dist) / (2.0f * maxDist));
    // dist large means distFcn close to 1
    // dist small means distFcn close to 0
}

double DelaunayGraphCut::facetMaxEdgeLength(Facet& f) const
{
    double dmax = 0.0;
    const Point3d& pa = _verticesCoords[getVertexIndex(f, 0)];
    const Point3d& pb = _verticesCoords[getVertexIndex(f, 1)];
    const Point3d& pc = _verticesCoords[getVertexIndex(f, 2)];
    double d0 = (pa - pb).size();
    double d1 = (pa - pc).size();
    double d2 = (pc - pb).size();
    dmax = std::max(dmax, d0);
    dmax = std::max(dmax, d1);
    dmax = std::max(dmax, d2);

    return dmax;
}

double DelaunayGraphCut::maxEdgeLength() const
{
    double dmax = 0.0f;

    for(int ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        // choose finite facet
        for(int k = 0; k < 4; ++k)
        {
            Facet facet(ci, k);
            dmax = std::max(dmax, facetMaxEdgeLength(facet));
        }
    }
    return dmax;
}

Point3d DelaunayGraphCut::cellCircumScribedSphereCentre(CellIndex ci) const
{
    // http://www.mps.mpg.de/homes/daly/CSDS/t4h/tetra.htm

    const Point3d r0 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
    const Point3d r1 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
    const Point3d r2 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
    const Point3d r3 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];

    const Point3d d1 = r1 - r0;
    const Point3d d2 = r2 - r0;
    const Point3d d3 = r3 - r0;

    float x = -(-(d1.x * d2.y * d3.z * conj(d1.x) - d1.x * d2.z * d3.y * conj(d1.x) + d1.y * d2.y * d3.z * conj(d1.y) -
                  d1.y * d2.z * d3.y * conj(d1.y) + d1.z * d2.y * d3.z * conj(d1.z) - d1.z * d2.z * d3.y * conj(d1.z) -
                  d1.y * d2.x * d3.z * conj(d2.x) + d1.z * d2.x * d3.y * conj(d2.x) - d1.y * d2.y * d3.z * conj(d2.y) +
                  d1.z * d2.y * d3.y * conj(d2.y) - d1.y * d2.z * d3.z * conj(d2.z) + d1.z * d2.z * d3.y * conj(d2.z) +
                  d1.y * d2.z * d3.x * conj(d3.x) - d1.z * d2.y * d3.x * conj(d3.x) + d1.y * d2.z * d3.y * conj(d3.y) -
                  d1.z * d2.y * d3.y * conj(d3.y) + d1.y * d2.z * d3.z * conj(d3.z) - d1.z * d2.y * d3.z * conj(d3.z)) /
                (2 * d1.x * d2.y * d3.z - 2 * d1.x * d2.z * d3.y - 2 * d1.y * d2.x * d3.z + 2 * d1.y * d2.z * d3.x +
                 2 * d1.z * d2.x * d3.y - 2 * d1.z * d2.y * d3.x));
    float y = -((d1.x * d2.x * d3.z * conj(d1.x) - d1.x * d2.z * d3.x * conj(d1.x) + d1.y * d2.x * d3.z * conj(d1.y) -
                 d1.y * d2.z * d3.x * conj(d1.y) + d1.z * d2.x * d3.z * conj(d1.z) - d1.z * d2.z * d3.x * conj(d1.z) -
                 d1.x * d2.x * d3.z * conj(d2.x) + d1.z * d2.x * d3.x * conj(d2.x) - d1.x * d2.y * d3.z * conj(d2.y) +
                 d1.z * d2.y * d3.x * conj(d2.y) - d1.x * d2.z * d3.z * conj(d2.z) + d1.z * d2.z * d3.x * conj(d2.z) +
                 d1.x * d2.z * d3.x * conj(d3.x) - d1.z * d2.x * d3.x * conj(d3.x) + d1.x * d2.z * d3.y * conj(d3.y) -
                 d1.z * d2.x * d3.y * conj(d3.y) + d1.x * d2.z * d3.z * conj(d3.z) - d1.z * d2.x * d3.z * conj(d3.z)) /
                (2 * d1.x * d2.y * d3.z - 2 * d1.x * d2.z * d3.y - 2 * d1.y * d2.x * d3.z + 2 * d1.y * d2.z * d3.x +
                 2 * d1.z * d2.x * d3.y - 2 * d1.z * d2.y * d3.x));
    float z = -(-(d1.x * d2.x * d3.y * conj(d1.x) - d1.x * d2.y * d3.x * conj(d1.x) + d1.y * d2.x * d3.y * conj(d1.y) -
                  d1.y * d2.y * d3.x * conj(d1.y) + d1.z * d2.x * d3.y * conj(d1.z) - d1.z * d2.y * d3.x * conj(d1.z) -
                  d1.x * d2.x * d3.y * conj(d2.x) + d1.y * d2.x * d3.x * conj(d2.x) - d1.x * d2.y * d3.y * conj(d2.y) +
                  d1.y * d2.y * d3.x * conj(d2.y) - d1.x * d2.z * d3.y * conj(d2.z) + d1.y * d2.z * d3.x * conj(d2.z) +
                  d1.x * d2.y * d3.x * conj(d3.x) - d1.y * d2.x * d3.x * conj(d3.x) + d1.x * d2.y * d3.y * conj(d3.y) -
                  d1.y * d2.x * d3.y * conj(d3.y) + d1.x * d2.y * d3.z * conj(d3.z) - d1.y * d2.x * d3.z * conj(d3.z)) /
                (2 * d1.x * d2.y * d3.z - 2 * d1.x * d2.z * d3.y - 2 * d1.y * d2.x * d3.z + 2 * d1.y * d2.z * d3.x +
                 2 * d1.z * d2.x * d3.y - 2 * d1.z * d2.y * d3.x));

    return r0 + Point3d(x, y, z);
}

// Returns a small score if one of the tetrahedon (from one side of the facet) is strange (the point in front of the current facet is far from the center).
// Returns the best value when the tetrahedons on both side of the facet are equilaterals.
double DelaunayGraphCut::getFaceWeight(const Facet& f1) const
{
    const Facet f2 = mirrorFacet(f1);
    const Point3d s1 = cellCircumScribedSphereCentre(f1.cellIndex);
    const Point3d s2 = cellCircumScribedSphereCentre(f2.cellIndex);

    const Point3d A = _verticesCoords[getVertexIndex(f1, 0)];
    const Point3d B = _verticesCoords[getVertexIndex(f1, 1)];
    const Point3d C = _verticesCoords[getVertexIndex(f1, 2)];

    const Point3d n = cross((B - A).normalize(), (C - A).normalize()).normalize();

    double a1 = fabs(angleBetwV1andV2(n, (A - s1).normalize()));
    if(a1 > 90)
    {
        a1 = 180.0 - a1;
    }

    double a2 = fabs(angleBetwV1andV2(n, (A - s2).normalize()));
    if(a2 > 90)
    {
        a2 = 180.0 - a2;
    }

    a1 = a1 * ((double)M_PI / 180.0);
    a2 = a2 * ((double)M_PI / 180.0);

    double wf = 1.0 - std::min(std::cos(a1), std::cos(a2));

    if((std::isnan(wf)) || (wf < 0.0f) || (wf > 1.0f))
        return 1.0;

    return wf;
}

float DelaunayGraphCut::weightFcn(float nrc, bool labatutWeights, int  /*ncams*/)
{
    float weight = 0.0f;
    if(labatutWeights)
    {
        weight = 32.0f;
    }
    else
    {
        weight = nrc;
    }
    return weight;
}

void DelaunayGraphCut::fillGraph(double nPixelSizeBehind, bool labatutWeights, bool fillOut, float distFcnHeight,
                                 float fullWeight) // nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0
                                                      // labatutWeights=0 fillOut=1 distFcnHeight=0
{
    ALICEVISION_LOG_INFO("Computing s-t graph weights.");
    long t1 = clock();

    // loop over all cells ... initialize
    for(GC_cellInfo& c: _cellsAttr)
    {
        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.fullnessScore = 0.0f;
        c.emptinessScore = 0.0f;
        c.on = 0.0f;
        for(int s = 0; s < 4; s++)
        {
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }

    // choose random order to prevent waiting
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    const std::vector<int> verticesRandIds = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size(), seed);

    int64_t totalStepsFront = 0;
    int64_t totalRayFront = 0;
    int64_t totalStepsBehind = 0;
    int64_t totalRayBehind = 0;

    size_t totalCamHaveVisibilityOnVertex = 0;
    size_t totalOfVertex = 0;

    size_t totalIsRealNrc = 0;
    
    GeometriesCount totalGeometriesIntersectedFrontCount;
    GeometriesCount totalGeometriesIntersectedBehindCount;

    boost::progress_display progressBar(std::min(size_t(100), verticesRandIds.size()), std::cout, "fillGraphPartPtRc\n");
    size_t progressStep = verticesRandIds.size() / 100;
    progressStep = std::max(size_t(1), progressStep);
#pragma omp parallel for reduction(+:totalStepsFront,totalRayFront,totalStepsBehind,totalRayBehind,totalCamHaveVisibilityOnVertex,totalOfVertex,totalIsRealNrc)
    for(int i = 0; i < verticesRandIds.size(); i++)
    {
        if(i % progressStep == 0)
        {
#pragma omp critical
            ++progressBar;
        }

        const int vertexIndex = verticesRandIds[i];
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];

        if(v.isReal())
        {
            ++totalIsRealNrc;
            // "weight" is called alpha(p) in the paper
            const float weight = weightFcn((float)v.nrc, labatutWeights, v.getNbCameras()); // number of cameras

            for(int c = 0; c < v.cams.size(); c++)
            {
                assert(v.cams[c] >= 0);
                assert(v.cams[c] < _mp.ncams);

                int stepsFront = 0;
                int stepsBehind = 0;
                GeometriesCount geometriesIntersectedFrontCount;
                GeometriesCount geometriesIntersectedBehindCount;
                fillGraphPartPtRc(stepsFront, stepsBehind, geometriesIntersectedFrontCount,
                                  geometriesIntersectedBehindCount, vertexIndex, v.cams[c], weight, fullWeight,
                                  nPixelSizeBehind,
                                  fillOut, distFcnHeight);

                totalStepsFront += stepsFront;
                totalRayFront += 1;
                totalStepsBehind += stepsBehind;
                totalRayBehind += 1;

#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedFrontCount.facets += geometriesIntersectedFrontCount.facets;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedFrontCount.vertices += geometriesIntersectedFrontCount.vertices;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedFrontCount.edges += geometriesIntersectedFrontCount.edges;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedBehindCount.facets += geometriesIntersectedBehindCount.facets;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedBehindCount.vertices += geometriesIntersectedBehindCount.vertices;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedBehindCount.edges += geometriesIntersectedBehindCount.edges;
            } // for c

            totalCamHaveVisibilityOnVertex += v.cams.size();
            totalOfVertex += 1;
        }
    }

    ALICEVISION_LOG_DEBUG("_verticesAttr.size(): " << _verticesAttr.size() << "(" << verticesRandIds.size() << ")");
    ALICEVISION_LOG_DEBUG("totalIsRealNrc: " << totalIsRealNrc);
    ALICEVISION_LOG_DEBUG("totalStepsFront//totalRayFront = " << totalStepsFront << " // " << totalRayFront);
    ALICEVISION_LOG_DEBUG("totalStepsBehind//totalRayBehind = " << totalStepsBehind << " // " << totalRayBehind);
    ALICEVISION_LOG_DEBUG("totalCamHaveVisibilityOnVertex//totalOfVertex = " << totalCamHaveVisibilityOnVertex << " // " << totalOfVertex);

    ALICEVISION_LOG_DEBUG("- Geometries Intersected count -");
    ALICEVISION_LOG_DEBUG("Front: " << totalGeometriesIntersectedFrontCount);
    ALICEVISION_LOG_DEBUG("Behind: " << totalGeometriesIntersectedBehindCount);
    totalGeometriesIntersectedFrontCount /= totalCamHaveVisibilityOnVertex;
    totalGeometriesIntersectedBehindCount /= totalCamHaveVisibilityOnVertex;
    ALICEVISION_LOG_DEBUG("Front per vertex: " << totalGeometriesIntersectedFrontCount);
    ALICEVISION_LOG_DEBUG("Behind per vertex: " << totalGeometriesIntersectedBehindCount);
    mvsUtils::printfElapsedTime(t1, "s-t graph weights computed : ");
}

void DelaunayGraphCut::fillGraphPartPtRc(
    int& outTotalStepsFront, int& outTotalStepsBehind, GeometriesCount& outFrontCount, GeometriesCount& outBehindCount,
    int vertexIndex, int cam, float weight, float fullWeight, double nPixelSizeBehind,
                                       bool fillOut, float distFcnHeight)  // nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 fillOut=1 distFcnHeight=0
{
    const int maxint = 1000000; // std::numeric_limits<int>::std::max()
    const double marginEpsilonFactor = 1.0e-4;

    const Point3d& originPt = _verticesCoords[vertexIndex];
    const double pixSize = _verticesAttr[vertexIndex].pixSize;
    const double maxDist = nPixelSizeBehind * pixSize;

    assert(cam >= 0);
    assert(cam < _mp.ncams);

    if(fillOut) // EMPTY part
    {
        // Initialisation
        GeometryIntersection geometry(vertexIndex); // Starting on global vertex index
        Point3d intersectPt = originPt;
        // toTheCam
        const double pointCamDistance = (_mp.CArr[cam] - originPt).size();
        const Point3d dirVect = (_mp.CArr[cam] - originPt).normalize();

#ifdef ALICEVISION_DEBUG_VOTE
        IntersectionHistory history(_mp.CArr[cam], originPt, dirVect);
#endif
        outTotalStepsFront = 0;
        Facet lastIntersectedFacet;
        bool lastGeoIsVertex = false;
        // Break only when we reach our camera vertex (as long as we find a next geometry)
        while(geometry.type != EGeometryType::Vertex || (_mp.CArr[cam] - intersectPt).size() >= 1.0e-3)
        {
            lastGeoIsVertex = false;
            // Keep previous informations
            const GeometryIntersection previousGeometry = geometry;
            const Point3d lastIntersectPt = intersectPt;

#ifdef ALICEVISION_DEBUG_VOTE
            history.append(geometry, intersectPt);
#endif
            ++outTotalStepsFront;

            geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

            if (geometry.type == EGeometryType::None)
            {
#ifdef ALICEVISION_DEBUG_VOTE
                // exportBackPropagationMesh("fillGraph_v" + std::to_string(vertexIndex) + "_ToCam_typeNone", history.geometries, originPt, _mp.CArr[cam]);
#endif
                // ALICEVISION_LOG_DEBUG(
                //     "[Error]: fillGraph(toTheCam) cause: geometry cannot be found."
                //     << "Current vertex index: " << vertexIndex
                //     << ", Previous geometry type: " << previousGeometry.type
                //     << ", outFrontCount:" << outFrontCount);
                break;
            }

            if((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
            {
                // Inverse direction, stop
                break;
            }

#ifdef ALICEVISION_DEBUG_VOTE
            {
                const auto end = history.geometries.end();
                auto it = std::find(history.geometries.begin(), end, geometry);
                if (it != end)
                {
                    // exportBackPropagationMesh("fillGraph_ToCam_alreadyIntersected", history.geometries, originPt, _mp.CArr[cam]);
                    ALICEVISION_LOG_DEBUG("[Error]: fillGraph(toTheCam) cause: intersected geometry has already been intersected.");
                    break;
                }
            }
#endif

            if (geometry.type == EGeometryType::Facet)
            {
                ++outFrontCount.facets;
                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[geometry.facet.cellIndex].emptinessScore += weight;
                }

                {
                    const float dist = distFcn(maxDist, (originPt - lastIntersectPt).size(), distFcnHeight);
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[geometry.facet.cellIndex].gEdgeVisWeight[geometry.facet.localVertexIndex] += weight * dist;
                }

                // Take the mirror facet to iterate over the next cell
                const Facet mFacet = mirrorFacet(geometry.facet);
                geometry.facet = mFacet;
                if (isInvalidOrInfiniteCell(mFacet.cellIndex))
                {
#ifdef ALICEVISION_DEBUG_VOTE
                    // exportBackPropagationMesh("fillGraph_ToCam_invalidMirorFacet", history.geometries, originPt, _mp.CArr[cam]);
#endif
                    //ALICEVISION_LOG_DEBUG("[Error]: fillGraph(toTheCam) cause: invalidOrInfinite miror facet.");
                    break;
                }
                lastIntersectedFacet = mFacet;
                if(previousGeometry.type == EGeometryType::Facet && outFrontCount.facets > 10000)
                {
                    ALICEVISION_LOG_WARNING("fillGraphPartPtRc front: loop on facets. Current landmark index: " << vertexIndex << ", camera: " << cam << ", outFrontCount: " << outFrontCount);
                    break;
                }
            }
            else
            {
                // We have just intersected a vertex or edge.
                // These geometries do not have a cellIndex, so we use the previousGeometry to retrieve the cell between the previous geometry and the current one.
                if (previousGeometry.type == EGeometryType::Facet)
                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[previousGeometry.facet.cellIndex].emptinessScore += weight;
                }

                if (geometry.type == EGeometryType::Vertex)
                {
                    ++outFrontCount.vertices;
                    lastGeoIsVertex = true;
                    if(previousGeometry.type == EGeometryType::Vertex && outFrontCount.vertices > 1000)
                    {
                        ALICEVISION_LOG_WARNING("fillGraphPartPtRc front: loop on vertices. Current landmark index: " << vertexIndex << ", camera: " << cam << ", outFrontCount: " << outFrontCount);
                        break;
                    }
                }
                else if (geometry.type == EGeometryType::Edge)
                {
                    ++outFrontCount.edges;
                    if(previousGeometry.type == EGeometryType::Edge && outFrontCount.edges > 1000)
                    {
                        ALICEVISION_LOG_WARNING("fillGraphPartPtRc front: loop on edges. Current landmark index: " << vertexIndex << ", camera: " << cam << ", outFrontCount: " << outFrontCount);
                        break;
                    }
                }
            }
            
            // Declare the last part of the empty path as connected to EMPTY (S node in the graph cut)
            if (lastIntersectedFacet.cellIndex != GEO::NO_CELL &&
                (_mp.CArr[cam] - intersectPt).size() < 0.2 * pointCamDistance)
            {
    #pragma OMP_ATOMIC_WRITE
                _cellsAttr[lastIntersectedFacet.cellIndex].cellSWeight = (float)maxint;
            }
        }

        // Vote for the last intersected facet (close to the cam)
        if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
        {
            // if(lastGeoIsVertex)
            {
                // lastGeoIsVertex is supposed to be positive in almost all cases.
                // If we do not reach the camera, we still vote on the last tetrehedra.
                // Possible reaisons: the camera is not part of the vertices or we encounter a numerical error in intersectNextGeom
#pragma OMP_ATOMIC_WRITE
                _cellsAttr[lastIntersectedFacet.cellIndex].cellSWeight = (float)maxint;
            }
            // else
            // {
            //     ALICEVISION_LOG_DEBUG(
            //         "fillGraph(toTheCam): last geometry is supposed to be the camera but it is not a vertex. "
            //         << "Current vertex index: " << vertexIndex
            //         << ", outFrontCount:" << outFrontCount);
            // }
        }
        // else
        // {
        //     ALICEVISION_LOG_DEBUG("fillGraph(toTheCam): no last intersected facet. "
        //                           << "Current vertex index: " << vertexIndex << ", outFrontCount:" << outFrontCount);
        // }
    }

    if(pixSize > 0.0f) // fillIn FULL part
    {
        const float fWeight = fullWeight * weight;
        // Initialisation
        GeometryIntersection geometry(vertexIndex); // Starting on global vertex index
        Point3d intersectPt = originPt;
        // behindThePoint
        const Point3d dirVect = (originPt - _mp.CArr[cam]).normalize();

#ifdef ALICEVISION_DEBUG_VOTE
        IntersectionHistory history(_mp.CArr[cam], originPt, dirVect);
#endif
        outTotalStepsBehind = 0;

        bool firstIteration = true;
        Facet lastIntersectedFacet;
        // While we are within the surface margin (as long as we find a next geometry)
        while ((originPt - intersectPt).size() < maxDist)
        {
            // Keep previous informations
            const GeometryIntersection previousGeometry = geometry;
            const Point3d lastIntersectPt = intersectPt;

#ifdef ALICEVISION_DEBUG_VOTE
            history.append(geometry, intersectPt);
#endif
            ++outTotalStepsBehind;

            geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

            if (geometry.type == EGeometryType::None)
            {
                // If we come from a facet, the next intersection must exist (even if the mirror facet is invalid, which is verified after taking mirror facet)
                if (previousGeometry.type == EGeometryType::Facet)
                {
#ifdef ALICEVISION_DEBUG_VOTE
                    // exportBackPropagationMesh("fillGraph_behindThePoint_NoneButPreviousIsFacet", history.geometries, originPt, _mp.CArr[cam]);
#endif
                    ALICEVISION_LOG_DEBUG("[Error]: fillGraph(behindThePoint) cause: None geometry but previous is Facet.");
                }
                // Break if we reach the end of the tetrahedralization volume
                break;
            }

            if((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
            {
                // Inverse direction, stop
                break;
            }
            if (geometry.type == EGeometryType::Facet)
            {
                ++outBehindCount.facets;

                // Vote for the first cell found (only once)
                if (firstIteration)
                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[geometry.facet.cellIndex].on += fWeight;
                    firstIteration = false;
                }

                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[geometry.facet.cellIndex].fullnessScore += fWeight;
                }

                // Take the mirror facet to iterate over the next cell
                const Facet mFacet = mirrorFacet(geometry.facet);
                lastIntersectedFacet = mFacet;
                geometry.facet = mFacet;
                if (isInvalidOrInfiniteCell(mFacet.cellIndex))
                {
                    // Break if we reach the end of the tetrahedralization volume (mirror facet cannot be found)
                    break;
                }

                {
                    const float dist = distFcn(maxDist, (originPt - lastIntersectPt).size(), distFcnHeight);
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[geometry.facet.cellIndex].gEdgeVisWeight[geometry.facet.localVertexIndex] +=
                        fWeight * dist;
                }
                if(previousGeometry.type == EGeometryType::Facet && outBehindCount.facets > 1000)
                {
                    ALICEVISION_LOG_WARNING("fillGraphPartPtRc behind: loop on facets. Current landmark index: " << vertexIndex << ", camera: " << cam << ", outBehindCount: " << outBehindCount);
                    break;
                }
            }
            else
            {
                // Vote for the first cell found (only once)
                // if we come from an edge or vertex to an other we have to vote for the first intersected cell.
                if (firstIteration)
                {
                    if (previousGeometry.type != EGeometryType::Vertex)
                    {
                        ALICEVISION_LOG_ERROR("[error] The firstIteration vote could only happen during for the first cell when we come from the first vertex.");
                        //throw std::runtime_error("[error] The firstIteration vote could only happen during for the first cell when we come from the first vertex.");
                    }
                    // the information of first intersected cell can only be found by taking intersection of neighbouring cells for both geometries
                    const std::vector<CellIndex> previousNeighbouring = getNeighboringCellsByVertexIndex(previousGeometry.vertexIndex);
                    const std::vector<CellIndex> currentNeigbouring = getNeighboringCellsByGeometry(geometry);

                    std::vector<CellIndex> neighboringCells;
                    std::set_intersection(previousNeighbouring.begin(), previousNeighbouring.end(), currentNeigbouring.begin(), currentNeigbouring.end(), std::back_inserter(neighboringCells));

                    for (const CellIndex& ci : neighboringCells)
                    {
#pragma OMP_ATOMIC_UPDATE
                        _cellsAttr[neighboringCells[0]].on += fWeight;
                    }
                    firstIteration = false;
                }

                // We have just intersected a vertex or edge.
                // These geometries do not have a cellIndex, so we use the previousGeometry to retrieve the cell between the previous geometry and the current one.
                if (previousGeometry.type == EGeometryType::Facet)
                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[previousGeometry.facet.cellIndex].fullnessScore += fWeight;
                }

                if (geometry.type == EGeometryType::Vertex)
                {
                    ++outBehindCount.vertices;
                    if(previousGeometry.type == EGeometryType::Vertex && outBehindCount.vertices > 1000)
                    {
                        ALICEVISION_LOG_WARNING("fillGraphPartPtRc behind: loop on vertices. Current landmark index: " << vertexIndex << ", camera: " << cam << ", outBehindCount: " << outBehindCount);
                        break;
                    }
                }
                else if (geometry.type == EGeometryType::Edge)
                {
                    ++outBehindCount.edges;
                    if(previousGeometry.type == EGeometryType::Edge && outBehindCount.edges > 1000)
                    {
                        ALICEVISION_LOG_WARNING("fillGraphPartPtRc behind: loop on edges. Current landmark index: " << vertexIndex << ", camera: " << cam << ", outBehindCount: " << outBehindCount);
                        break;
                    }
                }
            }
        }

        // cv: is the tetrahedron in distance 2*sigma behind the point p in the direction of the camera c (called Lcp in the paper) Vote for the last found facet
        // Vote for the last intersected facet (farthest from the camera)
        if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
        {
#pragma OMP_ATOMIC_UPDATE
            _cellsAttr[lastIntersectedFacet.cellIndex].cellTWeight += fWeight;
        }
    }
}

void DelaunayGraphCut::forceTedgesByGradientIJCV(float nPixelSizeBehind)
{
    ALICEVISION_LOG_INFO("Forcing t-edges");
    long t2 = clock();

    const float forceTEdgeDelta = (float)_mp.userParams.get<double>("delaunaycut.forceTEdgeDelta", 0.1f);
    ALICEVISION_LOG_DEBUG("forceTEdgeDelta: " << forceTEdgeDelta);

    const float minJumpPartRange = (float)_mp.userParams.get<double>("delaunaycut.minJumpPartRange", 10000.0f);
    ALICEVISION_LOG_DEBUG("minJumpPartRange: " << minJumpPartRange);

    const float maxSilentPartRange = (float)_mp.userParams.get<double>("delaunaycut.maxSilentPartRange", 100.0f);
    ALICEVISION_LOG_DEBUG("maxSilentPartRange: " << maxSilentPartRange);

    const float nsigmaJumpPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaJumpPart", 4.0f);
    ALICEVISION_LOG_DEBUG("nsigmaJumpPart: " << nsigmaJumpPart);

    const float nsigmaFrontSilentPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaFrontSilentPart", 2.0f);
    ALICEVISION_LOG_DEBUG("nsigmaFrontSilentPart: " << nsigmaFrontSilentPart);

    // This parameter allows to enlage the surface margin behind the point
    const float nsigmaBackSilentPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaBackSilentPart", 2.0f);
    ALICEVISION_LOG_DEBUG("nsigmaBackSilentPart: " << nsigmaBackSilentPart);

    for(GC_cellInfo& c: _cellsAttr)
    {
        c.on = 0.0f;
        // WARNING out is not the same as the sum because the sum are counted edges behind as well
        // c.out = c.gEdgeVisWeight[0] + c.gEdgeVisWeight[1] + c.gEdgeVisWeight[2] + c.gEdgeVisWeight[3];
    }

    const double marginEpsilonFactor = 1.0e-4;

    // choose random order to prevent waiting
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    const std::vector<int> verticesRandIds = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size(), seed);

    size_t totalStepsFront = 0;
    size_t totalRayFront = 0;
    size_t totalStepsBehind = 0;
    size_t totalRayBehind = 0;

    size_t totalCamHaveVisibilityOnVertex = 0;
    size_t totalOfVertex = 0;

    size_t totalVertexIsVirtual = 0;

    GeometriesCount totalGeometriesIntersectedFrontCount;
    GeometriesCount totalGeometriesIntersectedBehindCount;

#pragma omp parallel for reduction(+:totalStepsFront,totalRayFront,totalStepsBehind,totalRayBehind)
    for(int i = 0; i < verticesRandIds.size(); ++i)
    {
        const int vertexIndex = verticesRandIds[i];
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];
        if(v.isVirtual())
            continue;

        ++totalVertexIsVirtual;
        const Point3d& originPt = _verticesCoords[vertexIndex];
        // For each camera that has visibility over the vertex v (vertexIndex)
        for(const int cam : v.cams)
        {
            GeometriesCount geometriesIntersectedFrontCount;
            GeometriesCount geometriesIntersectedBehindCount;

            const float maxDist = nPixelSizeBehind * _mp.getCamPixelSize(originPt, cam);

            // float minJump = 10000000.0f;
            // float minSilent = 10000000.0f;
            float maxJump = 0.0f;
            float maxSilent = 0.0f;
            float midSilent = 10000000.0f;

            {
                // Initialisation
                GeometryIntersection geometry(vertexIndex); // Starting on global vertex index
                Point3d intersectPt = originPt;
                // toTheCam
                const Point3d dirVect = (_mp.CArr[cam] - originPt).normalize();

#ifdef ALICEVISION_DEBUG_VOTE
                IntersectionHistory history(_mp.CArr[cam], originPt, dirVect);
#endif
                // As long as we find a next geometry
                Point3d lastIntersectPt = originPt;
                // Iterate on geometries in the direction of camera's vertex within margin defined by maxDist (as long as we find a next geometry)
                while ((geometry.type != EGeometryType::Vertex || (_mp.CArr[cam] - intersectPt).size() > 1.0e-3) // We reach our camera vertex
                    && (lastIntersectPt - originPt).size() <= (nsigmaJumpPart + nsigmaFrontSilentPart) * maxDist) // We are to far from the originPt
                {
                    // Keep previous informations
                    const GeometryIntersection previousGeometry = geometry;
                    lastIntersectPt = intersectPt;

#ifdef ALICEVISION_DEBUG_VOTE
                    history.append(geometry, intersectPt);
#endif
                    ++totalStepsFront;

                    geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

                    if (geometry.type == EGeometryType::None)
                    {
#ifdef ALICEVISION_DEBUG_VOTE
                        // exportBackPropagationMesh("forceTedges_ToCam_typeNone", history.geometries, originPt, _mp.CArr[cam]);
#endif
                        // ALICEVISION_LOG_DEBUG("[Error]: forceTedges(toTheCam) cause: geometry cannot be found.");
                        break;
                    }

                    if((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
                    {
                        // Inverse direction, stop
                        break;
                    }
#ifdef ALICEVISION_DEBUG_VOTE
                    {
                        const auto end = history.geometries.end();
                        auto it = std::find(history.geometries.begin(), end, geometry);
                        if (it != end)
                        {
                            // exportBackPropagationMesh("forceTedges_ToCam_alreadyIntersected", history.geometries, originPt, _mp.CArr[cam]);
                            ALICEVISION_LOG_DEBUG("[Error]: forceTedges(toTheCam) cause: intersected geometry has already been intersected.");
                            break;
                        }
                    }
#endif

                    if (geometry.type == EGeometryType::Facet)
                    {
                        ++geometriesIntersectedFrontCount.facets;
                        const GC_cellInfo& c = _cellsAttr[geometry.facet.cellIndex];
                        if ((lastIntersectPt - originPt).size() > nsigmaFrontSilentPart * maxDist) // (p-originPt).size() > 2 * sigma
                        {
                            // minJump = std::min(minJump, c.emptinessScore);
                            maxJump = std::max(maxJump, c.emptinessScore);
                        }
                        else
                        {
                            // minSilent = std::min(minSilent, c.emptinessScore);
                            maxSilent = std::max(maxSilent, c.emptinessScore);
                        }

                        // Take the mirror facet to iterate over the next cell
                        const Facet mFacet = mirrorFacet(geometry.facet);
                        if (isInvalidOrInfiniteCell(mFacet.cellIndex))
                        {
#ifdef ALICEVISION_DEBUG_VOTE
                            // exportBackPropagationMesh("forceTedges_ToCam_invalidMirorFacet", history.geometries, originPt, _mp.CArr[cam]);
#endif
                            // ALICEVISION_LOG_DEBUG("[Error]: forceTedges(toTheCam) cause: invalidOrInfinite miror facet.");
                            break;
                        }
                        geometry.facet = mFacet;
                        if(previousGeometry.type == EGeometryType::Facet && geometriesIntersectedFrontCount.facets > 10000)
                        {
                            ALICEVISION_LOG_WARNING("forceTedgesByGradient front: loop on facets. Current landmark index: " << vertexIndex << ", camera: " << cam << ", intersectPt: " << intersectPt << ", lastIntersectPt: " << lastIntersectPt << ", geometriesIntersectedFrontCount: " << geometriesIntersectedFrontCount);
                            break;
                        }
                    }
                    else if (geometry.type == EGeometryType::Vertex)
                    {
                        ++geometriesIntersectedFrontCount.vertices;
                        if(previousGeometry.type == EGeometryType::Vertex && geometriesIntersectedFrontCount.vertices > 1000)
                        {
                            ALICEVISION_LOG_WARNING("forceTedgesByGradient front: loop on edges. Current landmark index: " << vertexIndex << ", camera: " << cam << ", geometriesIntersectedFrontCount: " << geometriesIntersectedFrontCount);
                            break;
                        }
                    }
                    else if (geometry.type == EGeometryType::Edge)
                    {
                        ++geometriesIntersectedFrontCount.edges;
                        if(previousGeometry.type == EGeometryType::Edge && geometriesIntersectedFrontCount.edges > 1000)
                        {
                            ALICEVISION_LOG_WARNING("forceTedgesByGradient front: loop on edges. Current landmark index: " << vertexIndex << ", camera: " << cam << ", geometriesIntersectedFrontCount: " << geometriesIntersectedFrontCount);
                            break;
                        }
                    }
                }
                ++totalRayFront;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedFrontCount.facets += geometriesIntersectedFrontCount.facets;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedFrontCount.vertices += geometriesIntersectedFrontCount.vertices;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedFrontCount.edges += geometriesIntersectedFrontCount.edges;
            }
            {
                // Initialisation
                GeometryIntersection geometry(vertexIndex);
                Point3d intersectPt = originPt;
                // behindThePoint
                const Point3d dirVect = (originPt - _mp.CArr[cam]).normalize();

#ifdef ALICEVISION_DEBUG_VOTE
                IntersectionHistory history(_mp.CArr[cam], originPt, dirVect);
#endif

                Facet lastIntersectedFacet;
                bool firstIteration = true;
		        Point3d lastIntersectPt = originPt;

                // While we are within the surface margin defined by maxDist (as long as we find a next geometry)
                while ((lastIntersectPt - originPt).size() <= nsigmaBackSilentPart * maxDist)
                {
                    // Keep previous informations
                    const GeometryIntersection previousGeometry = geometry;
                    lastIntersectPt = intersectPt;

#ifdef ALICEVISION_DEBUG_VOTE
                    history.append(geometry, intersectPt);
#endif
                    ++totalStepsBehind;

                    geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

                    if(geometry.type == EGeometryType::None)
                    {
//                         // If we come from a facet, the next intersection must exist (even if the mirror facet is invalid, which is verified later) 
//                         if (previousGeometry.type == EGeometryType::Facet)
//                         {
// #ifdef ALICEVISION_DEBUG_VOTE
//                             // exportBackPropagationMesh("forceTedges_behindThePoint_NoneButPreviousIsFacet", history.geometries, originPt, _mp.CArr[cam]);
// #endif
//                             ALICEVISION_LOG_DEBUG("[Error]: forceTedges(behindThePoint) cause: None geometry but previous is Facet.");
//                         }
                        // Break if we reach the end of the tetrahedralization volume
                        break;
                    }

                    if((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
                    {
                        // Inverse direction, stop
                        break;
                    }
                    if(geometry.type == EGeometryType::Facet)
                    {
                        ++geometriesIntersectedBehindCount.facets;

                        // Vote for the first cell found (only once)
                        if (firstIteration)
                        {
                            midSilent = _cellsAttr[geometry.facet.cellIndex].emptinessScore;
                            firstIteration = false;
                        }

                        const GC_cellInfo& c = _cellsAttr[geometry.facet.cellIndex];
                        // minSilent = std::min(minSilent, c.emptinessScore);
                        maxSilent = std::max(maxSilent, c.emptinessScore);

                        // Take the mirror facet to iterate over the next cell
                        const Facet mFacet = mirrorFacet(geometry.facet);
                        lastIntersectedFacet = mFacet;
                        geometry.facet = mFacet;
                        if (isInvalidOrInfiniteCell(mFacet.cellIndex))
                        {
                            // Break if we reach the end of the tetrahedralization volume (mirror facet cannot be found)
                            break;
                        }
                        if(previousGeometry.type == EGeometryType::Facet && geometriesIntersectedBehindCount.facets > 1000)
                        {
                            ALICEVISION_LOG_WARNING("forceTedgesByGradient behind: loop on facets. Current landmark index: " << vertexIndex << ", camera: " << cam << ", geometriesIntersectedBehindCount: " << geometriesIntersectedBehindCount);
                            break;
                        }
                    }
                    else
                    {
                        // Vote for the first cell found (only once)
                        // if we come from an edge or vertex to an other we have to vote for the first intersected cell.
                        if (firstIteration)
                        {
                            if (previousGeometry.type != EGeometryType::Vertex)
                            {
                                ALICEVISION_LOG_ERROR("The firstIteration vote could only happen during for "
                                                      "the first cell when we come from the first vertex.");
                                // throw std::runtime_error("[error] The firstIteration vote could only happen during for the first cell when we come from the first vertex.");
                            }
                            // the information of first intersected cell can only be found by taking intersection of neighbouring cells for both geometries
                            const std::vector<CellIndex> previousNeighbouring = getNeighboringCellsByVertexIndex(previousGeometry.vertexIndex);
                            const std::vector<CellIndex> currentNeigbouring = getNeighboringCellsByGeometry(geometry);

                            std::vector<CellIndex> neighboringCells;
                            std::set_intersection(previousNeighbouring.begin(), previousNeighbouring.end(), currentNeigbouring.begin(), currentNeigbouring.end(), std::back_inserter(neighboringCells));

                            for (const CellIndex& ci : neighboringCells)
                            {
                                midSilent = _cellsAttr[geometry.facet.cellIndex].emptinessScore;
                            }
                            firstIteration = false;
                        }

                        if (geometry.type == EGeometryType::Vertex)
                        {
                            ++geometriesIntersectedBehindCount.vertices;
                            if(previousGeometry.type == EGeometryType::Vertex && geometriesIntersectedBehindCount.vertices > 1000)
                            {
                                ALICEVISION_LOG_WARNING("forceTedgesByGradient behind: loop on vertices. Current landmark index: " << vertexIndex << ", camera: " << cam << ", geometriesIntersectedBehindCount: " << geometriesIntersectedBehindCount);
                                break;
                            }
                        }
                        else if (geometry.type == EGeometryType::Edge)
                        {
                            ++geometriesIntersectedBehindCount.edges;
                            if(previousGeometry.type == EGeometryType::Edge && geometriesIntersectedBehindCount.edges > 1000)
                            {
                                ALICEVISION_LOG_WARNING("forceTedgesByGradient behind: loop on edges. Current landmark index: " << vertexIndex << ", camera: " << cam << ", geometriesIntersectedBehindCount: " << geometriesIntersectedBehindCount);
                                break;
                            }
                        }
                    }
                }

                if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
                {
                    // Equation 6 in paper
                    //   (g / B) < k_rel
                    //   (B - g) > k_abs
                    //   g < k_outl

                    // In the paper:
                    // B (beta): max value before point p
                    // g (gamma): mid-range score behind point p

                    // In the code:
                    // maxJump: max score of emptiness in all the tetrahedron along the line of sight between camera c and 2*sigma before p
                    // midSilent: score of the next tetrahedron directly after p (called T1 in the paper)
                    // maxSilent: max score of emptiness for the tetrahedron around the point p (+/- 2*sigma around p)

                    if((midSilent / maxJump < forceTEdgeDelta) && // (g / B) < k_rel    //// k_rel=0.1
                       (maxJump - midSilent > minJumpPartRange) && // (B - g) > k_abs   //// k_abs=10000 // 1000 in the paper
                       (maxSilent < maxSilentPartRange)) // g < k_outl                  //// k_outl=100  // 400 in the paper
                        //(maxSilent-minSilent<maxSilentPartRange))
                    {
#pragma OMP_ATOMIC_UPDATE
                        _cellsAttr[lastIntersectedFacet.cellIndex].on += (maxJump - midSilent);
                    }
                }
                ++totalRayBehind;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedBehindCount.facets += totalGeometriesIntersectedBehindCount.facets;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedBehindCount.vertices += totalGeometriesIntersectedBehindCount.vertices;
#pragma OMP_ATOMIC_UPDATE
                totalGeometriesIntersectedBehindCount.edges += totalGeometriesIntersectedBehindCount.edges;
            }
        }
        totalCamHaveVisibilityOnVertex += v.cams.size();
        totalOfVertex += 1;
    }

    for(GC_cellInfo& c: _cellsAttr)
    {
        const float w = std::max(1.0f, c.cellTWeight) * c.on;

        // cellTWeight = clamp(w, cellTWeight, 1000000.0f);
        c.cellTWeight = std::max(c.cellTWeight, std::min(1000000.0f, w));
    }

    ALICEVISION_LOG_DEBUG("_verticesAttr.size(): " << _verticesAttr.size() << "(" << verticesRandIds.size() << ")");
    ALICEVISION_LOG_DEBUG("totalVertexIsVirtual: " << totalVertexIsVirtual);
    ALICEVISION_LOG_DEBUG("totalStepsFront//totalRayFront = " << totalStepsFront << " // " << totalRayFront);
    ALICEVISION_LOG_DEBUG("totalStepsBehind//totalRayBehind = " << totalStepsBehind << " // " << totalRayBehind);
    ALICEVISION_LOG_DEBUG("totalCamHaveVisibilityOnVertex//totalOfVertex = " << totalCamHaveVisibilityOnVertex << " // " << totalOfVertex);

    ALICEVISION_LOG_DEBUG("- Geometries Intersected count -");
    ALICEVISION_LOG_DEBUG("Front: " << totalGeometriesIntersectedFrontCount);
    ALICEVISION_LOG_DEBUG("Behind: " << totalGeometriesIntersectedBehindCount);
    totalGeometriesIntersectedFrontCount /= totalRayFront;
    totalGeometriesIntersectedBehindCount /= totalRayBehind;
    ALICEVISION_LOG_DEBUG("Front per vertex: " << totalGeometriesIntersectedFrontCount);
    ALICEVISION_LOG_DEBUG("Behind per vertex: " << totalGeometriesIntersectedBehindCount);

    mvsUtils::printfElapsedTime(t2, "t-edges forced: ");
}

int DelaunayGraphCut::computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const
{
    vertexIsOnSurface.resize(_verticesCoords.size(), false);

    int nbSurfaceFacets = 0;
    // loop over all facets
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            const Facet f1(ci, k);
            const bool uo = _cellIsFull[f1.cellIndex]; // get if it is occupied
            if(!uo)
                continue;

            const Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            const bool vo = _cellIsFull[f2.cellIndex]; // get if it is occupied

            if(vo)
                continue;

            VertexIndex v1 = getVertexIndex(f1, 0);
            VertexIndex v2 = getVertexIndex(f1, 1);
            VertexIndex v3 = getVertexIndex(f1, 2);
            ++nbSurfaceFacets;
            vertexIsOnSurface[v1] = true;
            vertexIsOnSurface[v2] = true;
            vertexIsOnSurface[v3] = true;

            assert(!(isInfiniteCell(f1.cellIndex) && isInfiniteCell(f2.cellIndex))); // infinite both cells of finite vertex!
        }
    }
    ALICEVISION_LOG_INFO("computeIsOnSurface nbSurfaceFacets: " << nbSurfaceFacets);
    return nbSurfaceFacets;
}

void DelaunayGraphCut::graphCutPostProcessing(const Point3d hexah[8], const std::string& folderName)
{
    long timer = std::clock();
    ALICEVISION_LOG_INFO("Graph cut post-processing.");

    int minSegmentSize = _mp.userParams.get<int>("hallucinationsFiltering.minSegmentSize", 10);
    bool doRemoveBubbles = _mp.userParams.get<bool>("hallucinationsFiltering.doRemoveBubbles", true);
    bool doRemoveDust = _mp.userParams.get<bool>("hallucinationsFiltering.doRemoveDust", true);
    bool doLeaveLargestFullSegmentOnly = _mp.userParams.get<bool>("hallucinationsFiltering.doLeaveLargestFullSegmentOnly", false);
    int invertTetrahedronBasedOnNeighborsNbIterations = _mp.userParams.get<bool>("hallucinationsFiltering.invertTetrahedronBasedOnNeighborsNbIterations", 10);
    double minSolidAngleRatio = _mp.userParams.get<double>("hallucinationsFiltering.minSolidAngleRatio", 0.2);
    int nbSolidAngleFilteringIterations = _mp.userParams.get<double>("hallucinationsFiltering.nbSolidAngleFilteringIterations", 10);

    if(doRemoveBubbles)
    {
        // Remove empty spaces that are not connected to at least one camera
        removeBubbles();
    }

    {
        std::size_t nbFullCells = std::accumulate(_cellIsFull.begin(), _cellIsFull.end(), 0);
        ALICEVISION_LOG_INFO("[" << __LINE__ << "] Nb full cells: " << nbFullCells << " / " << _cellIsFull.size() << " cells.");
    }

    if(true)
    {
        // free all full cell that have a camera vertex
        int nbModifiedCells = 0;
        for(int rc = 0; rc < _mp.ncams; rc++)
        {
            VertexIndex cam_vi = _camsVertexes[rc];
            if(cam_vi == GEO::NO_VERTEX)
                continue;
            for(CellIndex adjCellIndex : getNeighboringCellsByVertexIndex(cam_vi)) // GEOGRAM: set_stores_cicl(true) required
            {
                if(isInvalidOrInfiniteCell(adjCellIndex))
                    continue;

                if(_cellIsFull[adjCellIndex])
                {
                    _cellIsFull[adjCellIndex] = false;
                    ++nbModifiedCells;
                }
            }
        }
        ALICEVISION_LOG_WARNING("Declare empty around camera centers: " << nbModifiedCells << " cells changed to empty within " << _cellIsFull.size() << " cells.");
    }


    // Set cells that have a point outside hexahedron as empty
    if(hexah != nullptr)
    {
        int nbModifiedCells = 0;
        Point3d hexahinf[8];
        mvsUtils::inflateHexahedron(hexah, hexahinf, 1.001);
        for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
        {
            if(isInvalidOrInfiniteCell(ci) || !_cellIsFull[ci])
                continue;

            const Point3d& pa = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
            const Point3d& pb = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
            const Point3d& pc = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
            const Point3d& pd = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];

            if((!mvsUtils::isPointInHexahedron(pa, hexahinf)) ||
               (!mvsUtils::isPointInHexahedron(pb, hexahinf)) ||
               (!mvsUtils::isPointInHexahedron(pc, hexahinf)) ||
               (!mvsUtils::isPointInHexahedron(pd, hexahinf)))
            {
                _cellIsFull[ci] = false;
                ++nbModifiedCells;
            }
        }
        ALICEVISION_LOG_WARNING("Full cells with a vertex outside the BBox are changed to empty: " << nbModifiedCells << " cells changed to empty.");
    }

    if(doRemoveDust)
    {
        removeDust(minSegmentSize);
    }

    if(doLeaveLargestFullSegmentOnly)
    {
        leaveLargestFullSegmentOnly();
    }

    if(saveTemporaryBinFiles)
    {
        saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }

    if(_mp.userParams.get<bool>("LargeScale.saveDelaunayTriangulation", false))
    {
      const std::string fileNameDh = folderName + "delaunayTriangulation.bin";
      const std::string fileNameInfo = folderName + "delaunayTriangulationInfo.bin";
      saveDh(fileNameDh, fileNameInfo);
    }

    invertFullStatusForSmallLabels();

    {
        // Changed status of cells to improve coherence with neighboring tetrahedrons
        // If 3 or 4 facets are connected to cells of the opporite status,
        // it is better to update the current status.
        for(int i = 0; i < invertTetrahedronBasedOnNeighborsNbIterations; ++i)
        {
            StaticVector<CellIndex> toDoInverse;
            toDoInverse.reserve(_cellIsFull.size());

            for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
            {
                int count = 0;
                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    count += (_cellIsFull[nci] != _cellIsFull[ci]);
                }
                if(count > 2)
                    toDoInverse.push_back(ci);
            }
            if(toDoInverse.empty())
                break;
            int movedToEmpty = 0;
            int movedToFull = 0;
            for(std::size_t i = 0; i < toDoInverse.size(); ++i)
            {
                CellIndex ci = toDoInverse[i];
                _cellIsFull[ci] = !_cellIsFull[ci];
                if(_cellIsFull[ci])
                    ++movedToFull;
                else
                    ++movedToEmpty;
            }
            ALICEVISION_LOG_WARNING("[" << i << "] Coherence with neighboring tetrahedrons: "
                << movedToFull << " cells moved to full, "
                << movedToEmpty << " cells moved to empty within "
                << _cellIsFull.size() << " cells.");
        }
    }
    
    cellsStatusFilteringBySolidAngleRatio(nbSolidAngleFilteringIterations, minSolidAngleRatio);

    ALICEVISION_LOG_INFO("Graph cut post-processing done.");

    mvsUtils::printfElapsedTime(timer, "Graph cut post-processing ");
}

void DelaunayGraphCut::invertFullStatusForSmallLabels()
{
    ALICEVISION_LOG_DEBUG("filling small holes");

    const std::size_t nbCells = _cellIsFull.size();
    StaticVector<int> colorPerCell(nbCells, -1);

    StaticVector<int> nbCellsPerColor;
    nbCellsPerColor.reserve(100);
    nbCellsPerColor.resize_with(1, 0);
    int lastColorId = 0;

    StaticVector<CellIndex> buff;
    buff.reserve(nbCells);

    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        if(colorPerCell[ci] == -1)
        {
            // backtrack all connected interior cells
            buff.resize(0);
            buff.push_back(ci);

            colorPerCell[ci] = lastColorId;
            nbCellsPerColor[lastColorId] += 1;

            while(buff.size() > 0)
            {
                CellIndex tmp_ci = buff.pop();

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    if((colorPerCell[nci] == -1) && (_cellIsFull[nci] == _cellIsFull[ci]))
                    {
                        colorPerCell[nci] = lastColorId;
                        nbCellsPerColor[lastColorId] += 1;
                        buff.push_back(nci);
                    }
                }
            }
            nbCellsPerColor.push_back(0); // add new color with 0 cell
            ++lastColorId;
            assert(lastColorId == nbCellsPerColor.size() - 1);
        }
    }
    assert(nbCellsPerColor[nbCellsPerColor.size()-1] == 0);
    nbCellsPerColor.resize(nbCellsPerColor.size()-1); // remove last empty element

    int movedToEmpty = 0;
    int movedToFull = 0;
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        if(nbCellsPerColor[colorPerCell[ci]] < 100)
        {
            _cellIsFull[ci] = !_cellIsFull[ci];
            if(_cellIsFull[ci])
                ++movedToFull;
            else
                ++movedToEmpty;
        }
    }

    ALICEVISION_LOG_WARNING("DelaunayGraphCut::invertFullStatusForSmallLabels: "
        << movedToFull << " cells moved to full, "
        << movedToEmpty << " cells moved to empty within "
        << _cellIsFull.size() << " cells.");

    ALICEVISION_LOG_DEBUG("Number of labels: " << nbCellsPerColor.size() << ", Number of cells changed: " << movedToFull + movedToEmpty << ", full number of cells: " << nbCells);
}

void DelaunayGraphCut::cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations,
                                                             double minSolidAngleRatio)
{
    if(nbSolidAngleFilteringIterations <= 0 || minSolidAngleRatio <= 0.0)
        return;

    constexpr double fullSphereSolidAngle = 4.0 * boost::math::constants::pi<double>();

    // Change cells status on surface around vertices to improve smoothness
    // using solid angle ratio between full/empty parts.
    for(int i = 0; i < nbSolidAngleFilteringIterations; ++i)
    {
        std::vector<bool> cellsInvertStatus(_cellIsFull.size(), false);
        int toInvertCount = 0;

        std::vector<bool> vertexIsOnSurface;
        const int nbSurfaceFacets = computeIsOnSurface(vertexIsOnSurface);

#pragma omp parallel for reduction(+ : toInvertCount)
        for(int vi = 0; vi < _neighboringCellsPerVertex.size(); ++vi)
        {
            if(!vertexIsOnSurface[vi])
                continue;
            // ALICEVISION_LOG_INFO("vertex is on surface: " << vi);
            const std::vector<CellIndex>& neighboringCells = _neighboringCellsPerVertex[vi];
            std::vector<Facet> neighboringFacets;
            neighboringFacets.reserve(neighboringCells.size());
            bool borderCase = false;
            double fullPartSolidAngle = 0.0;
            for(CellIndex ci : neighboringCells)
            {
                // ALICEVISION_LOG_INFO("full cell: " << ci);
                std::vector<VertexIndex> triangle;
                triangle.reserve(3);
                GEO::signed_index_t localVertexIndex = 0;
                for(int k = 0; k < 4; ++k)
                {
                    const GEO::signed_index_t currentVertex = _tetrahedralization->cell_vertex(ci, k);
                    if(currentVertex == GEO::NO_VERTEX)
                        break;
                    if(currentVertex != vi)
                        triangle.push_back(currentVertex);
                    else
                        localVertexIndex = k;
                }
                if(triangle.size() != 3)
                {
                    borderCase = true;
                    break;
                }
                {
                    const Facet f(ci, localVertexIndex);
                    neighboringFacets.push_back(f);
                }

                if (_cellIsFull[ci])
                {
                    const Point3d& O = _verticesCoords[vi];
                    const double s =
                        tetrahedronSolidAngle(_verticesCoords[triangle[0]] - O, _verticesCoords[triangle[1]] - O,
                                              _verticesCoords[triangle[2]] - O);
                    // ALICEVISION_LOG_INFO("tetrahedronSolidAngle: " << s);
                    fullPartSolidAngle += s;
                }
            }
            if(borderCase)
            {
                // we cannot compute empty/full ratio if we have undefined cells
                continue;
            }
            bool invert = false;
            bool invertFull = false;
            // ALICEVISION_LOG_INFO("fullPartSolidAngle: " << fullPartSolidAngle << ", ratio: "
            //                                             << fullPartSolidAngle / fullSphereSolidAngle);
            if(fullPartSolidAngle < minSolidAngleRatio * fullSphereSolidAngle)
            {
                invert = true;
                invertFull = true; // we want to invert the FULL cells
            }
            else if(fullPartSolidAngle > (1.0 - minSolidAngleRatio) * fullSphereSolidAngle)
            {
                invert = true;
                invertFull = false; // we want to invert the EMPTY cells
            }
            if(!invert)
                continue;

            // Ensure that we do not increase inconsitencies (like holes).
            // Check the status coherency with neighbor cells if we swap the cells status.
            for(const Facet& f : neighboringFacets)
            {
                if(_cellIsFull[f.cellIndex] == invertFull)
                {
                    const Facet fv = mirrorFacet(f);
                    if(isInvalidOrInfiniteCell(fv.cellIndex) || _cellIsFull[f.cellIndex] != _cellIsFull[fv.cellIndex])
                    {
                        borderCase = true;
                        break;
                    }
                }
            }
            if(borderCase)
                continue;

            // Invert some cells
            for(CellIndex ci : neighboringCells)
            {
                if(_cellIsFull[ci] == invertFull)
                {
#pragma OMP_ATOMIC_WRITE
                    cellsInvertStatus[ci] = true;
                    ++toInvertCount;
                }
            }
        }
        if(toInvertCount == 0)
            break;
        int movedToEmpty = 0;
        int movedToFull = 0;
        for(CellIndex ci = 0; ci < cellsInvertStatus.size(); ++ci)
        {
            if(!cellsInvertStatus[ci])
                continue;
            _cellIsFull[ci] = !_cellIsFull[ci];
            if(_cellIsFull[ci])
                ++movedToFull;
            else
                ++movedToEmpty;
        }
        ALICEVISION_LOG_WARNING("[" << i << "] Check solid angle full/empty ratio on surface vertices: " << movedToFull
                                    << " cells moved to full, " << movedToEmpty << " cells moved to empty within "
                                    << _cellIsFull.size() << " cells.");
    }
}

void DelaunayGraphCut::createDensePointCloud(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData* sfmData, const FuseParams* depthMapsFuseParams)
{
  assert(sfmData != nullptr || depthMapsFuseParams != nullptr);

  ALICEVISION_LOG_INFO("Creating dense point cloud.");

  const float minDist = hexah ? (hexah[0] - hexah[1]).size() / 1000.0f : 0.00001f;
  const int helperPointsGridSize = _mp.userParams.get<int>("LargeScale.helperPointsGridSize", 10);
  const int densifyNbFront = _mp.userParams.get<int>("LargeScale.densifyNbFront", 0);
  const int densifyNbBack = _mp.userParams.get<int>("LargeScale.densifyNbBack", 0);
  const double densifyScale = _mp.userParams.get<double>("LargeScale.densifyScale", 1.0);

  // add points from depth maps
  if(depthMapsFuseParams != nullptr)
    fuseFromDepthMaps(cams, hexah, *depthMapsFuseParams);

  // add points from sfm
  if(sfmData != nullptr)
    addPointsFromSfM(hexah, cams, *sfmData);

  // add points for cam centers
  addPointsFromCameraCenters(cams, minDist);

  // add 6 points to prevent singularities
  addPointsToPreventSingularities(hexah, minDist);

  densifyWithHelperPoints(densifyNbFront, densifyNbBack, densifyScale);

  // add volume points to prevent singularities
  {
    Point3d hexahExt[8];
    mvsUtils::inflateHexahedron(hexah, hexahExt, 1.1);
    addGridHelperPoints(helperPointsGridSize, hexahExt, minDist);

    // add point for shape from silhouette
    if(depthMapsFuseParams != nullptr)
      addMaskHelperPoints(hexahExt, cams, *depthMapsFuseParams);
  }

  _verticesCoords.shrink_to_fit();
  _verticesAttr.shrink_to_fit();

  ALICEVISION_LOG_WARNING("Final dense point cloud: " << _verticesCoords.size() << " points.");
}

void DelaunayGraphCut::createGraphCut(const Point3d hexah[8], const StaticVector<int>& cams,
                                      const std::string& folderName, const std::string& tmpCamsPtsFolderName,
                                      bool removeSmallSegments, bool exportDebugTetrahedralization)
{
  // Create tetrahedralization
  computeDelaunay();
  displayStatistics();

  if(removeSmallSegments) // false
  {
    std::vector<GC_Seg> segments;
    std::vector<bool> useVertex; // keep empty to process all pixels
    computeVerticesSegSize(segments, useVertex, 0.0f);
    removeSmallSegs(segments, 2500); // TODO FACA: to decide
  }

  voteFullEmptyScore(cams, folderName);

  if(exportDebugTetrahedralization)
    exportFullScoreMeshs(folderName, "");

  maxflow();
}

void DelaunayGraphCut::addToInfiniteSw(float sW)
{
    std::size_t nbInfinitCells = 0;
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        if(isInfiniteCell(ci))
        {
            GC_cellInfo& c = _cellsAttr[ci];
            c.cellSWeight += sW;
            ++nbInfinitCells;
        }
    }
    ALICEVISION_LOG_WARNING("DelaunayGraphCut::addToInfiniteSw nbInfinitCells: " << nbInfinitCells);
}

void DelaunayGraphCut::maxflow()
{
    long t_maxflow = clock();

    ALICEVISION_LOG_INFO("Maxflow: start allocation.");
    const std::size_t nbCells = _cellsAttr.size();
    ALICEVISION_LOG_INFO("Number of cells: " << nbCells);

    // MaxFlow_CSR maxFlowGraph(nbCells);
    MaxFlow_AdjList maxFlowGraph(nbCells);

    ALICEVISION_LOG_INFO("Maxflow: add nodes.");
    // fill s-t edges
    int nbSCells = 0;
    int nbTCells = 0;
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        const GC_cellInfo& c = _cellsAttr[ci];
        const float ws = c.cellSWeight;
        const float wt = c.cellTWeight;

        assert(ws >= 0.0f);
        assert(wt >= 0.0f);
        assert(!std::isnan(ws));
        assert(!std::isnan(wt));

        maxFlowGraph.addNode(ci, ws, wt);
        if(ws > wt)
            ++nbSCells;
        else
            ++nbTCells;
    }
    ALICEVISION_LOG_INFO("Maxflow: " << nbSCells << " S cells, " << nbTCells << " T cells.");

    ALICEVISION_LOG_INFO("Maxflow: add edges.");
    const float CONSTalphaVIS = 1.0f;
    const float CONSTalphaPHOTO = 5.0f;

    // fill u-v directed edges
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet fu(ci, k);
            Facet fv = mirrorFacet(fu);
            if(isInvalidOrInfiniteCell(fv.cellIndex))
                continue;

            float a1 = 0.0f;
            float a2 = 0.0f;
            if((!isInfiniteCell(fu.cellIndex)) && (!isInfiniteCell(fv.cellIndex)))
            {
                // Score for each facet based on the quality of the topology
                a1 = getFaceWeight(fu);
                a2 = getFaceWeight(fv);
            }

            // In output of maxflow the cuts will become the surface.
            // High weight on some facets will avoid cutting them.
            float wFvFu = _cellsAttr[fu.cellIndex].gEdgeVisWeight[fu.localVertexIndex] * CONSTalphaVIS + a1 * CONSTalphaPHOTO;
            float wFuFv = _cellsAttr[fv.cellIndex].gEdgeVisWeight[fv.localVertexIndex] * CONSTalphaVIS + a2 * CONSTalphaPHOTO;

            assert(wFvFu >= 0.0f);
            assert(wFuFv >= 0.0f);
            assert(!std::isnan(wFvFu));
            assert(!std::isnan(wFuFv));

            maxFlowGraph.addEdge(fu.cellIndex, fv.cellIndex, wFuFv, wFvFu);
        }
    }

    ALICEVISION_LOG_INFO("Maxflow: clear cells info.");
    std::vector<GC_cellInfo>().swap(_cellsAttr); // force clear to free some RAM before maxflow

    long t_maxflow_compute = clock();
    // Find graph-cut solution
    ALICEVISION_LOG_INFO("Maxflow: compute.");
    const float totalFlow = maxFlowGraph.compute();
    mvsUtils::printfElapsedTime(t_maxflow_compute, "Maxflow computation ");
    ALICEVISION_LOG_INFO("totalFlow: " << totalFlow);

    ALICEVISION_LOG_INFO("Maxflow: update full/empty cells status.");
    _cellIsFull.resize(nbCells);
    // Update FULL/EMPTY status of all cells
    std::size_t nbFullCells = 0;
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        _cellIsFull[ci] = maxFlowGraph.isTarget(ci);
        nbFullCells += _cellIsFull[ci];
    }
    ALICEVISION_LOG_WARNING("Maxflow full/nbCells: " << nbFullCells << " / " << nbCells);

    mvsUtils::printfElapsedTime(t_maxflow, "Full maxflow step");

    ALICEVISION_LOG_INFO("Maxflow: done.");
}

void DelaunayGraphCut::voteFullEmptyScore(const StaticVector<int>& cams, const std::string& folderName)
{
    ALICEVISION_LOG_INFO("DelaunayGraphCut::voteFullEmptyScore");
    const int maxint = 1000000.0f;

    long t1;

    // TODO FACA: nPixelSizeBehind 2 or 4 by default?
    const double nPixelSizeBehind = _mp.userParams.get<double>("delaunaycut.nPixelSizeBehind", 4.0); // sigma value
    const float fullWeight = float(_mp.userParams.get<double>("delaunaycut.fullWeight", 1.0));

    ALICEVISION_LOG_INFO("nPixelSizeBehind: " << nPixelSizeBehind);

    // 0 for distFcn equals 1 all the time
    const float distFcnHeight = (float)_mp.userParams.get<double>("delaunaycut.distFcnHeight", 0.0);

    const bool labatutCFG09 = _mp.userParams.get<bool>("global.LabatutCFG09", false);
    // jancosekIJCV: "Exploiting Visibility Information in Surface Reconstruction to Preserve Weakly Supported Surfaces", Michal Jancosek and Tomas Pajdla, 2014
    const bool jancosekIJCV = _mp.userParams.get<bool>("global.JancosekIJCV", true);

    if(jancosekIJCV) // true by default
    {
        const bool forceTEdge = _mp.userParams.get<bool>("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);

        displayCellsStats();

        // compute weights on edge between tetrahedra
        fillGraph(nPixelSizeBehind, false, true, distFcnHeight, fullWeight);

        displayCellsStats();

        addToInfiniteSw((float)maxint);

        displayCellsStats();

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoInit.bin");

        if(false)
        {
            std::unique_ptr<mesh::Mesh> meshf(createTetrahedralMesh(false, 0.9f, [](const fuseCut::GC_cellInfo& c) { return c.emptinessScore; }));
            meshf->save(folderName + "tetrahedralMesh_beforeForceTEdge_emptiness");
        }

        if(forceTEdge)
        {
            forceTedgesByGradientIJCV(nPixelSizeBehind);
        }

        displayCellsStats();

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterForce.bin");
    }

    if(labatutCFG09)
    {
        ALICEVISION_LOG_INFO("Labatut CFG 2009 method:");
        fillGraph(nPixelSizeBehind, true, true, distFcnHeight, fullWeight);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoInit.bin");
    }
}

void DelaunayGraphCut::filterLargeHelperPoints(std::vector<bool>& out_reliableVertices,
                                               const std::vector<bool>& vertexIsOnSurface, int maxSegSize)
{
    ALICEVISION_LOG_DEBUG("DelaunayGraphCut::filterLargeHelperPoints");
    out_reliableVertices.clear();

    // Do not filter helper points if maxSegSize is negative/infinit
    if(maxSegSize < 0)
        return;

    out_reliableVertices.resize(_verticesAttr.size(), true);

    std::vector<Pixel> edges;
    edges.reserve(_verticesAttr.size());

    // Create edges for all connected helper points
    for(VertexIndex vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        const GC_vertexInfo& v = _verticesAttr[vi];
        const Point3d& p = _verticesCoords[vi];
        if(v.nrc <= 0 && vertexIsOnSurface[vi])
        {
            // go through all the neighbouring points
            GEO::vector<VertexIndex> adjVertices;
            _tetrahedralization->get_neighbors(vi, adjVertices);

            for(VertexIndex nvi : adjVertices)
            {
                // ignore itself
                if(vi == nvi)
                    continue;
                // avoid duplicates
                if(vi < nvi)
                    continue;
                // ignore points not on the surface
                if(!vertexIsOnSurface[vi])
                    continue;
                // ignore valid vertices
                const GC_vertexInfo& nv = _verticesAttr[nvi];
                if(nv.nrc > 0)
                    continue;
                // Declare a new edge between 2 helper points both on the selected surface
                edges.emplace_back(vi, nvi);
            }
        }
    }

    Universe u(_verticesAttr.size());

    if (maxSegSize > 0)
    {
        int s = (int)edges.size();
        // Fuse all edges collected to be merged
        for(int i = 0; i < s; i++)
        {
            int a = u.find(edges[i].x);
            int b = u.find(edges[i].y);
            if(a != b)
            {
                u.join(a, b);
            }
        }
    }

    // Last loop over vertices to update segId
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        if(!vertexIsOnSurface[vi])
        {
            // Point is not on the surface
            out_reliableVertices[vi] = false;
            continue;
        }
        const GC_vertexInfo& v = _verticesAttr[vi];
        if(v.nrc > 0)
        {
            // This is not a helper point, so it is reliable.
            out_reliableVertices[vi] = true;
            continue;
        }

        if(maxSegSize > 0)
        {
            // It is an helper point, so it is reliable only if it is a small group.
            const int sigId = u.find(vi);
            const int segSize = u.elts[sigId].size;
            out_reliableVertices[vi] = segSize <= maxSegSize;
        }
        else
        {
            // It is an helper point and should be removed.
            out_reliableVertices[vi] = false;
        }
    }

    ALICEVISION_LOG_DEBUG("DelaunayGraphCut::filterLargeHelperPoints done.");
}

mesh::Mesh* DelaunayGraphCut::createMesh(int maxNbConnectedHelperPoints)
{
    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut.");

    std::vector<bool> vertexIsOnSurface;
    const int nbSurfaceFacets = computeIsOnSurface(vertexIsOnSurface);

    ALICEVISION_LOG_INFO("# surface facets: " << nbSurfaceFacets);
    ALICEVISION_LOG_INFO("# vertixes: " << _verticesCoords.size());
    ALICEVISION_LOG_INFO("_cellIsFull.size(): " << _cellIsFull.size());

    mesh::Mesh* me = new mesh::Mesh();

    // TODO: copy only surface points and remap visibilities
    me->pts = StaticVector<Point3d>();
    me->pts.reserve(_verticesCoords.size());

    for(const Point3d& p: _verticesCoords)
    {
        me->pts.push_back(p);
    }

    std::vector<bool> reliableVertices;
    filterLargeHelperPoints(reliableVertices, vertexIsOnSurface, maxNbConnectedHelperPoints);

    me->tris = StaticVector<mesh::Mesh::triangle>();
    me->tris.reserve(nbSurfaceFacets);

    // loop over all facets
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellIsFull[f1.cellIndex]; // get if it is occupied
            if(!uo)
            {
                // "f1" is in an EMPTY cell, skip it
                continue;
            }

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellIsFull[f2.cellIndex]; // get if it is occupied

            if(vo)
            {
                // "f2" is in a FULL cell, skip it
                continue;
            }

            // "f1" is in a FULL cell and "f2" is in an EMPTY cell

            VertexIndex vertices[3];
            vertices[0] = getVertexIndex(f1, 0);
            vertices[1] = getVertexIndex(f1, 1);
            vertices[2] = getVertexIndex(f1, 2);

            if(!reliableVertices.empty())
            {
                // We skip triangles if it contains one unreliable vertex.
                bool invalidTriangle = false;
                for(int k = 0; k < 3; ++k)
                {
                    if(!reliableVertices[vertices[k]])
                    {
                        invalidTriangle = true;
                        break;
                    }
                }
                if(invalidTriangle)
                    continue;
            }

            Point3d points[3];
            for(int k = 0; k < 3; ++k)
            {
                points[k] = _verticesCoords[vertices[k]];
            }

            // if(T.is_infinite(f1.first->vertex(f1.second)))
            // {
            //    printf("WARNINIG infinite vertex\n");
            // }

            const Point3d D1 = _verticesCoords[getOppositeVertexIndex(f1)]; // in FULL part
            const Point3d D2 = _verticesCoords[getOppositeVertexIndex(f2)]; // in EMPTY part

            const Point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

            const double dd1 = orientedPointPlaneDistance(D1, points[0], N);
            // const double dd2 = orientedPointPlaneDistance(D2, points[0], N);

            const bool clockwise = std::signbit(dd1);

            if(clockwise)
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertices[0];
                t.v[1] = vertices[1];
                t.v[2] = vertices[2];
                me->tris.push_back(t);
            }
            else
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertices[0];
                t.v[1] = vertices[2];
                t.v[2] = vertices[1];
                me->tris.push_back(t);
            }
        }
    }

    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut done.");
    return me;
}

void DelaunayGraphCut::displayCellsStats() const
{
    ALICEVISION_LOG_INFO("DelaunayGraphCut::displayCellsStats");

    using namespace boost::accumulators;
    using Accumulator = accumulator_set<float, stats<tag::min, tag::max, tag::median, tag::mean>>;
    auto displayAcc = [](const std::string& name, const Accumulator& acc) {
        ALICEVISION_LOG_INFO(" [" << name << "]"
                                  << " min: " << extract::min(acc) << " max: " << extract::max(acc)
                                  << " mean: " << extract::mean(acc) << " median: " << extract::median(acc));
        };
    {
        Accumulator acc_nrc;
        Accumulator acc_camSize;
        for(VertexIndex vi = 0; vi < _verticesAttr.size(); ++vi)
        {
            const GC_vertexInfo& vertexAttr = _verticesAttr[vi];
            acc_nrc(vertexAttr.nrc);
            acc_camSize(vertexAttr.cams.size());
        }
        displayAcc("acc_nrc", acc_nrc);
        displayAcc("acc_camSize", acc_camSize);
    }
    {
        Accumulator acc_cellScore;
        Accumulator acc_cellSWeight;
        Accumulator acc_cellTWeight;
        Accumulator acc_gEdgeVisWeight;
        Accumulator acc_fullnessScore;
        Accumulator acc_emptinessScore;
        Accumulator acc_on;
        int64_t countPositiveSWeight = 0;

        for(const GC_cellInfo& cellAttr : _cellsAttr)
        {
            countPositiveSWeight += (cellAttr.cellSWeight > 0);
            acc_cellScore(cellAttr.cellSWeight - cellAttr.cellTWeight);
            acc_cellSWeight(cellAttr.cellSWeight);
            acc_cellTWeight(cellAttr.cellTWeight);

            acc_gEdgeVisWeight(cellAttr.gEdgeVisWeight[0]);
            acc_gEdgeVisWeight(cellAttr.gEdgeVisWeight[1]);
            acc_gEdgeVisWeight(cellAttr.gEdgeVisWeight[2]);
            acc_gEdgeVisWeight(cellAttr.gEdgeVisWeight[3]);

            acc_fullnessScore(cellAttr.fullnessScore);
            acc_emptinessScore(cellAttr.emptinessScore);
            acc_on(cellAttr.on);
        }

        displayAcc("cellScore", acc_cellScore);
        displayAcc("cellSWeight", acc_cellSWeight);
        displayAcc("cellTWeight", acc_cellTWeight);
        displayAcc("gEdgeVisWeight", acc_gEdgeVisWeight);
        displayAcc("fullnessScore", acc_fullnessScore);
        displayAcc("emptinessScore", acc_emptinessScore);
        displayAcc("on", acc_on);
        ALICEVISION_LOG_INFO("countPositiveSWeight: " << countPositiveSWeight);
    }
}

mesh::Mesh* DelaunayGraphCut::createTetrahedralMesh(bool filter, const float& downscaleFactor, const std::function<float(const GC_cellInfo&)> getScore) const
{
    ALICEVISION_LOG_INFO("Create mesh of the tetrahedralization.");

    ALICEVISION_LOG_INFO("# vertices: " << _verticesCoords.size());

    mesh::Mesh* me = new mesh::Mesh();
    if(_cellsAttr.empty())
    {
        ALICEVISION_LOG_INFO("Empty tetrahedralization.");
        return me;
    }

    // TODO: copy only surface points and remap visibilities
    me->pts.reserve(10 * _verticesCoords.size());

    me->tris.reserve(_verticesCoords.size());

    using namespace boost::accumulators;
    using Accumulator = accumulator_set<float, stats<tag::min, tag::max, tag::median, tag::mean>>;
    auto displayAcc = [](const std::string& name, const Accumulator& acc) {
        ALICEVISION_LOG_INFO(" [" << name << "]"
                                  << " min: " << extract::min(acc) << " max: " << extract::max(acc)
                                  << " mean: " << extract::mean(acc) << " median: " << extract::median(acc));
    };
    float maxScore = 1.0;
    {
        Accumulator acc_selectedScore;

        for(const GC_cellInfo& cellAttr : _cellsAttr)
        {
            acc_selectedScore(getScore(cellAttr));
        }
        displayAcc("selected", acc_selectedScore);
        maxScore = 4.0f * extract::mean(acc_selectedScore);
    }

    ALICEVISION_LOG_DEBUG("createTetrahedralMesh: maxScore: " << maxScore);

    // Prevent division by zero (getRGBFromJetColorMap)
    if (maxScore == 0)
        maxScore = 1;

    // loop over all facets
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        Point3d pointscellCenter(0.0, 0.0, 0.0);
        {
            bool invalid = false;
            int weakVertex = 0;
            for(VertexIndex k = 0; k < 4; ++k)
            {
                const VertexIndex vi = _tetrahedralization->cell_vertex(ci, k);
                const GC_vertexInfo& vertexAttr = _verticesAttr[vi];
                if(filter && vertexAttr.cams.size() <= 3)
                {
                    ++weakVertex;
                }
                const Facet f1(ci, k);
                const Facet f2 = mirrorFacet(f1);
                if(filter && isInvalidOrInfiniteCell(f2.cellIndex))
                {
                    invalid = true;
                    continue;
                }
                pointscellCenter = pointscellCenter + _verticesCoords[vi];
            }
            pointscellCenter = pointscellCenter / 4.0;
            if(filter && invalid)
            {
                // If one of the mirror facet is invalid, discard the tetrahedron.
                continue;
            }
            if (filter && (weakVertex >= 3))
            {
                // If the tetrahedron mainly composed of weak vertices, skip it.
                // So keep tetrahedra that are linked to at least 3 good vertices.
                continue;
            }
        }

        const GC_cellInfo& cellAttr = _cellsAttr[ci];
        const float score = getScore(cellAttr); // cellAttr.cellSWeight - cellAttr.cellTWeight;
        if(filter && (score < (maxScore / 1000.0f)))
        {
            // Skip too low score cells
            continue;
        }

        for(VertexIndex k = 0; k < 4; ++k)
        {
            const Facet f1(ci, k);

            VertexIndex vertices[3];
            vertices[0] = getVertexIndex(f1, 0);
            vertices[1] = getVertexIndex(f1, 1);
            vertices[2] = getVertexIndex(f1, 2);

            Point3d points[3];
            for(int k = 0; k < 3; ++k)
            {
                points[k] = _verticesCoords[vertices[k]];
                // Downscale cell for visibility
                points[k] = pointscellCenter + ((points[k] - pointscellCenter) * downscaleFactor);
            }

            const Facet f2 = mirrorFacet(f1);
            bool clockwise = false;
            
            //// do not need to test again: already filtered before
            if (!isInvalidOrInfiniteCell(f2.cellIndex))
            {
                const Point3d D1 = _verticesCoords[getOppositeVertexIndex(f1)];
                const Point3d D2 = _verticesCoords[getOppositeVertexIndex(f2)];

                const Point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

                const double dd1 = orientedPointPlaneDistance(D1, points[0], N);
                const double dd2 = orientedPointPlaneDistance(D2, points[0], N);
                if (dd1 == 0.0)
                {
                    if (dd2 == 0.0)
                    {
                        ALICEVISION_LOG_WARNING("createMesh: bad triangle orientation.");
                    }
                    if (dd2 > 0.0)
                    {
                        clockwise = true;
                    }
                }
                else
                {
                    if (dd1 < 0.0)
                    {
                        clockwise = true;
                    }
                }
            }

            const rgb color = getRGBFromJetColorMap(score / maxScore);
            const std::size_t vertexBaseIndex = me->pts.size();
            for(const Point3d& p: points)
            {
                me->pts.push_back(p);
                me->colors().push_back(color);
            }

            if(clockwise)
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertexBaseIndex;
                t.v[1] = vertexBaseIndex + 1;
                t.v[2] = vertexBaseIndex + 2;
                me->tris.push_back(t);
            }
            else
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertexBaseIndex;
                t.v[1] = vertexBaseIndex + 2;
                t.v[2] = vertexBaseIndex + 1;
                me->tris.push_back(t);
            }
        }
    }

    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut done.");
    return me;
}

void DelaunayGraphCut::exportDebugMesh(const std::string& filename, const Point3d& fromPt, const Point3d& toPt)
{
    std::unique_ptr<mesh::Mesh> mesh(createTetrahedralMesh(false, 0.999f));
    std::unique_ptr<mesh::Mesh> meshf(createTetrahedralMesh(true, 0.999f));

    // hack add direction vector from fromPt to toPt
    {
        const Point3d dirVec = fromPt - toPt;
        const Point3d deltaVec = (dirVec * 0.1f);

        mesh->pts.push_back(toPt - deltaVec);
        mesh->colors().push_back(rgb(255, 0, 0));
        mesh->pts.push_back(fromPt + deltaVec);
        mesh->colors().push_back(rgb(0, 0, 255));
        mesh->pts.push_back(fromPt + deltaVec * 0.001f);
        mesh->colors().push_back(rgb(0, 0, 255));

        meshf->pts.push_back(toPt - deltaVec);
        meshf->colors().push_back(rgb(255, 0, 0));
        meshf->pts.push_back(fromPt + deltaVec);
        meshf->colors().push_back(rgb(0, 0, 255));
        meshf->pts.push_back(fromPt + deltaVec * 0.001f);
        meshf->colors().push_back(rgb(0, 0, 255));
    }
    {
        mesh::Mesh::triangle t;
        t.alive = true;
        t.v[0] = meshf->pts.size() - 3;
        t.v[1] = meshf->pts.size() - 2;
        t.v[2] = meshf->pts.size() - 1;
        meshf->tris.push_back(t);
    }
    {
        mesh::Mesh::triangle t;
        t.alive = true;
        t.v[0] = mesh->pts.size() - 3;
        t.v[1] = mesh->pts.size() - 2;
        t.v[2] = mesh->pts.size() - 1;
        mesh->tris.push_back(t);
    }

    const std::string tempDirPath = boost::filesystem::temp_directory_path().generic_string();
    mesh->save(tempDirPath + "/" + filename);
    meshf->save(tempDirPath + "/" + filename);
}

void DelaunayGraphCut::exportFullScoreMeshs(const std::string& outputFolder, const std::string& name) const
{
    const std::string nameExt = (name.empty() ? "" : "_" + name);
    {
        std::unique_ptr<mesh::Mesh> meshEmptiness(
            createTetrahedralMesh(false, 0.999f, [](const GC_cellInfo& c) { return c.emptinessScore; }));
        meshEmptiness->save(outputFolder + "/mesh_emptiness" + nameExt);
    }
    {
        std::unique_ptr<mesh::Mesh> meshFullness(
            createTetrahedralMesh(false, 0.999f, [](const GC_cellInfo& c) { return c.fullnessScore; }));
        meshFullness->save(outputFolder + "/mesh_fullness" + nameExt);
    }
    {
        std::unique_ptr<mesh::Mesh> meshSWeight(
            createTetrahedralMesh(false, 0.999f, [](const GC_cellInfo& c) { return c.cellSWeight; }));
        meshSWeight->save(outputFolder + "/mesh_sWeight" + nameExt);
    }
    {
        std::unique_ptr<mesh::Mesh> meshTWeight(
            createTetrahedralMesh(false, 0.999f, [](const GC_cellInfo& c) { return c.cellTWeight; }));
        meshTWeight->save(outputFolder + "/mesh_tWeight" + nameExt);
    }
    {
        std::unique_ptr<mesh::Mesh> meshOn(
            createTetrahedralMesh(false, 0.999f, [](const GC_cellInfo& c) { return c.on; }));
        meshOn->save(outputFolder + "/mesh_on" + nameExt);
    }
    {
        std::unique_ptr<mesh::Mesh> mesh(createTetrahedralMesh(
            false, 0.99f, [](const fuseCut::GC_cellInfo& c) { return c.fullnessScore - c.emptinessScore; }));
        mesh->save(outputFolder + "/mesh_fullness-emptiness" + nameExt);
    }
    {
        std::unique_ptr<mesh::Mesh> mesh(createTetrahedralMesh(
            false, 0.99f, [](const fuseCut::GC_cellInfo& c) { return c.cellSWeight - c.cellTWeight; }));
        mesh->save(outputFolder + "/mesh_s-t" + nameExt);
    }
}

void DelaunayGraphCut::exportBackPropagationMesh(const std::string& filename, std::vector<GeometryIntersection>& intersectedGeom, const Point3d& fromPt, const Point3d& toPt)
{
    // Clean _cellsAttr emptinessScore
    for (int i = 0; i < _cellsAttr.size(); ++i)
        _cellsAttr[i].emptinessScore = 0.0f;

    // Vote only for listed intersected geom facets
    for (size_t i = 0; i < intersectedGeom.size(); i++)
    {
        const GeometryIntersection& geo = intersectedGeom[i];

        if (geo.type == EGeometryType::Facet)
            _cellsAttr[geo.facet.cellIndex].emptinessScore += i;
    }

    exportDebugMesh(filename + "_BackProp", fromPt, toPt);
}

void DelaunayGraphCut::writeScoreInCsv(const std::string& filePath, const size_t& sizeLimit)
{
    assert(boost::filesystem::path(filePath).extension().string() == std::string(".csv"));

    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    std::mt19937 generator(seed != 0 ? seed : std::random_device{}());

    std::vector<int> idx(_cellsAttr.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::shuffle(idx.begin(), idx.end(), generator);

    std::ofstream csv(filePath);
    const char sep = ','; // separator
    csv << "fullnessScore" << sep <<
        "emptinessScore" << sep <<
        "cellSWeight" << sep <<
        "cellTWeight" << sep <<
        "on" << sep <<
        "gEdgeVisWeight0" << sep <<
        "gEdgeVisWeight1" << sep <<
        "gEdgeVisWeight2" << sep <<
        "gEdgeVisWeight3" << '\n';
    const size_t size = sizeLimit > 0 ? std::min(sizeLimit, _cellsAttr.size()) : _cellsAttr.size();
    for (size_t i = 0; i < size; ++i)
    {
        const GC_cellInfo& cellAttr = _cellsAttr[idx.back()];
        idx.pop_back();
        csv << cellAttr.fullnessScore << sep <<
            cellAttr.emptinessScore << sep <<
            cellAttr.cellSWeight << sep <<
            cellAttr.cellTWeight << sep <<
            cellAttr.on << sep <<
            cellAttr.gEdgeVisWeight[0] << sep <<
            cellAttr.gEdgeVisWeight[1] << sep <<
            cellAttr.gEdgeVisWeight[2] << sep <<
            cellAttr.gEdgeVisWeight[3] << '\n';
    }

    csv.close();
    ALICEVISION_LOG_INFO("Csv exported: " << filePath);
}

void DelaunayGraphCut::segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& out_nsegments)
{
    ALICEVISION_LOG_DEBUG("segmentFullOrFree: segmenting connected space.");

    out_fullSegsColor.clear();
    out_fullSegsColor.reserve(_cellIsFull.size());
    out_fullSegsColor.resize_with(_cellIsFull.size(), -1);

    StaticVector<CellIndex> buff;
    buff.reserve(_cellIsFull.size());
    int col = 0;

    // segment connected free space
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if((!isInfiniteCell(ci)) && (out_fullSegsColor[ci] == -1) && (_cellIsFull[ci] == full))
        {
            // backtrack all connected interior cells
            buff.resize(0);
            buff.push_back(ci);

            while(buff.size() > 0)
            {
                CellIndex tmp_ci = buff.pop();

                out_fullSegsColor[tmp_ci] = col;

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    if((!isInfiniteCell(nci)) && (out_fullSegsColor[nci] == -1) &&
                       (_cellIsFull[nci] == full))
                    {
                        buff.push_back(nci);
                    }
                }
            }
            ++col;
        }
    }

    out_nsegments = col;
}

int DelaunayGraphCut::removeBubbles()
{
    ALICEVISION_LOG_DEBUG("Removing empty bubbles.");

    int nbEmptySegments = 0;
    StaticVector<int> emptySegColors;
    segmentFullOrFree(false, emptySegColors, nbEmptySegments);

    StaticVectorBool colorsToFill;
    colorsToFill.reserve(nbEmptySegments);
    // all free space segments which contains camera has to remain free all others full
    colorsToFill.resize_with(nbEmptySegments, true);

    // all free space segments which contains camera has to remain free
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if(isInfiniteCell(ci) || emptySegColors[ci] < 0)
            continue;

        const GC_vertexInfo& a = _verticesAttr[_tetrahedralization->cell_vertex(ci, 0)];
        const GC_vertexInfo& b = _verticesAttr[_tetrahedralization->cell_vertex(ci, 1)];
        const GC_vertexInfo& c = _verticesAttr[_tetrahedralization->cell_vertex(ci, 2)];
        const GC_vertexInfo& d = _verticesAttr[_tetrahedralization->cell_vertex(ci, 3)];
        if( a.isVirtual() || b.isVirtual() || c.isVirtual() || d.isVirtual())
        {
            // TODO FACA: check helper points are not connected to cameras?
            colorsToFill[emptySegColors[ci]] = false;
        }
    }

    int nbBubbles = 0;
    for(int i = 0; i < nbEmptySegments; ++i)
    {
        if(colorsToFill[i])
        {
            ++nbBubbles;
        }
    }

    int nbModifiedCells = 0;
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if((!isInfiniteCell(ci)) && (emptySegColors[ci] >= 0) && (colorsToFill[emptySegColors[ci]]))
        {
            _cellIsFull[ci] = true;
            ++nbModifiedCells;
        }
    }

    ALICEVISION_LOG_INFO("DelaunayGraphCut::removeBubbles: nbBubbles: " << nbBubbles << ", all empty segments: " << nbEmptySegments);
    ALICEVISION_LOG_WARNING("DelaunayGraphCut::removeBubbles: " << nbModifiedCells << " cells changed to full within " << _cellIsFull.size() << " cells.");

    return nbBubbles;
}

int DelaunayGraphCut::removeDust(int minSegSize)
{
    ALICEVISION_LOG_DEBUG("Removing dust (isolated full cells).");

    int nbFullSegments = 0;
    StaticVector<int> fullSegsColor;
    segmentFullOrFree(true, fullSegsColor, nbFullSegments);

    StaticVector<int> colorsSize(nbFullSegments, 0);

    // all free space segments which contains camera has to remain free
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if(fullSegsColor[ci] >= 0) // if we have a valid color: non empty and non infinit cell
        {
            colorsSize[fullSegsColor[ci]] += 1; // count the number of cells in the segment
        }
    }

    int ndust = 0;
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        // if number of cells in the segment is too small, we change the status to "empty"
        if((fullSegsColor[ci] >= 0) && (colorsSize[fullSegsColor[ci]] < minSegSize))
        {
            _cellIsFull[ci] = false;
            ++ndust;
        }
    }

    ALICEVISION_LOG_INFO("DelaunayGraphCut::removeDust: Number of full segments: " << nbFullSegments);
    ALICEVISION_LOG_WARNING("DelaunayGraphCut::removeDust: " << ndust << " cells changed to empty within " << _cellIsFull.size() << " cells.");

    return ndust;
}

void DelaunayGraphCut::leaveLargestFullSegmentOnly()
{
    ALICEVISION_LOG_DEBUG("Largest full segment only.");

    int nsegments;
    StaticVector<int> colors;
    segmentFullOrFree(true, colors, nsegments);

    StaticVector<int> colorsSize(nsegments, 0);

    // all free space segments which contains camera has to remain free
    int largestColor = -1;
    int maxn = 0;
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        int color = colors[ci];
        if(color >= 0)
        {
            colorsSize[color] += 1;
            int n = colorsSize[color];
            if(n > maxn)
            {
                maxn = n;
                largestColor = color;
            }
        }
    }

    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if(colors[ci] != largestColor)
        {
            _cellIsFull[ci] = false;
        }
    }

    ALICEVISION_LOG_DEBUG("Largest full segment only done.");
}

std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::EGeometryType type)
{
    switch (type)
    {
    case DelaunayGraphCut::EGeometryType::Facet:
        stream << "Facet";
        break;
    case DelaunayGraphCut::EGeometryType::Vertex:
        stream << "Vertex";
        break;
    case DelaunayGraphCut::EGeometryType::Edge:
        stream << "Edge";
        break;
    case DelaunayGraphCut::EGeometryType::None:
        stream << "None";
        break;
    }
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::Facet& facet)
{
    stream << "c:" << facet.cellIndex << ",v:" << facet.localVertexIndex;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::Edge& edge)
{
    stream << "v0:" << edge.v0 << ",v1:" << edge.v1;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::GeometryIntersection& intersection)
{
    stream << intersection.type << ": ";
    switch (intersection.type)
    {
    case DelaunayGraphCut::EGeometryType::Facet:
        stream << intersection.facet;
        break;
    case DelaunayGraphCut::EGeometryType::Vertex:
        stream << intersection.vertexIndex;
        break;
    case DelaunayGraphCut::EGeometryType::Edge:
        stream << intersection.edge;
        break;
    case DelaunayGraphCut::EGeometryType::None:
        stream << "None";
        break;
    }
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::GeometriesCount& count)
{
    stream << "facets:" << count.facets << ",edges:" << count.edges << ",vertices:" << count.vertices;
    return stream;
}

} // namespace fuseCut
} // namespace aliceVision
