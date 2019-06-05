// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

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
#include <aliceVision/alicevision_omp.hpp>

#include "nanoflann.hpp"

#include <geogram/points/kd_tree.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

// OpenMP >= 3.1 for advanced atomic clauses (https://software.intel.com/en-us/node/608160)
// OpenMP preprocessor version: https://github.com/jeffhammond/HPCInfo/wiki/Preprocessor-Macros
#if defined _OPENMP && _OPENMP >= 201107 
#define OMP_ATOMIC_UPDATE _Pragma("omp atomic update")
#define OMP_ATOMIC_WRITE  _Pragma("omp atomic write")
#define OMP_HAVE_MIN_MAX_REDUCTION
#else
#define OMP_ATOMIC_UPDATE _Pragma("omp atomic")
#define OMP_ATOMIC_WRITE  _Pragma("omp atomic")
#endif

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
                                    std::vector<GC_vertexInfo>& verticesAttrPrepare, mvsUtils::MultiViewParams* mp, float simFactor, float voteMarginFactor, float contributeMarginFactor, float simGaussianSize)
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
            imageIO::readImage(simMapFilepath, wTmp, hTmp, simMap, imageIO::EImageColorSpace::NO_CONVERSION);
            if(wTmp != width || hTmp != height)
                throw std::runtime_error("Similarity map size doesn't match the depth map size: " + simMapFilepath + ", " + depthMapFilepath);
            {
                std::vector<float> simMapTmp(simMap.size());
                imageIO::convolveImage(width, height, simMap, simMapTmp, "gaussian", simGaussianSize, simGaussianSize);
                simMap.swap(simMapTmp);
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

                const Point3d p = mp->backproject(c, Point2d(x, y), depth);
                const double pixSize = mp->getCamPixelSize(p, c);
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

//    verticesCoordsPrepare.swap(newVerticesCoordsPrepare);
//    simScorePrepare.swap(newSimScorePrepare);
//    pixSizePrepare.swap(newPixSizePrepare);
    ALICEVISION_LOG_INFO("Visibilities created.");
}


DelaunayGraphCut::DelaunayGraphCut(mvsUtils::MultiViewParams* _mp)
{
    mp = _mp;

    _camsVertexes.resize(mp->ncams, -1);

    saveTemporaryBinFiles = mp->userParams.get<bool>("LargeScale.saveTemporaryBinFiles", false);

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

void DelaunayGraphCut::initVertices()
{
    ALICEVISION_LOG_DEBUG("initVertices ...\n");

    // Re-assign ids to the vertices to go one after another
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        GC_vertexInfo& v = _verticesAttr[vi];
        // fit->info().point = convertPointToPoint3d(fit->point());
        v.isOnSurface = false;
        // v.id = nVertices;
        v.pixSize = mp->getCamsMinPixelSize(_verticesCoords[vi], v.cams);
    }

    ALICEVISION_LOG_DEBUG("initVertices done\n");
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
    ALICEVISION_LOG_DEBUG("initCells ...\n");

    _cellsAttr.resize(_tetrahedralization->nb_cells()); // or nb_finite_cells() if keeps_infinite()

    ALICEVISION_LOG_INFO(_cellsAttr.size() << " cells created by tetrahedralization.");
    for(int i = 0; i < _cellsAttr.size(); ++i)
    {
        GC_cellInfo& c = _cellsAttr[i];

        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.on = 0.0f;
        c.in = 0.0f;
        c.out = 0.0f;
        for(int s = 0; s < 4; ++s)
        {
            c.gEdgeVisWeight[s] = 0.0f; // weights for the 4 faces of the tetrahedron
        }
    }

    ALICEVISION_LOG_DEBUG("initCells done\n");
}

void DelaunayGraphCut::displayStatistics()
{
    // Display some statistics

    StaticVector<int>* ptsCamsHist = getPtsCamsHist();
    ALICEVISION_LOG_INFO("Histogram of number of cams per point:");
    for(int i = 0; i < ptsCamsHist->size(); ++i)
        ALICEVISION_LOG_INFO("    " << i << ": " << mvsUtils::num2str((*ptsCamsHist)[i]));
    delete ptsCamsHist;

    StaticVector<int>* ptsNrcsHist = getPtsNrcHist();
    ALICEVISION_LOG_INFO("Histogram of Nrc per point:");
    for(int i = 0; i < ptsNrcsHist->size(); ++i)
        ALICEVISION_LOG_INFO("    " << i << ": " << mvsUtils::num2str((*ptsNrcsHist)[i]));
    delete ptsNrcsHist;
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
    cams.resize_with(mp->getNbCameras(), 0);

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
        vAttrIt->cams.push_back(mp->getIndexFromViewId(observationPair.first));

      ++vCoordsIt;
      ++vAttrIt;
    }
    ++landmarkIt;
  }

  _verticesCoords.shrink_to_fit();
  _verticesAttr.shrink_to_fit();
}

void DelaunayGraphCut::addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist)
{
    for(int camid = 0; camid < cams.size(); camid++)
    {
        int rc = cams[camid];
        {
            Point3d p(mp->CArr[rc].x, mp->CArr[rc].y, mp->CArr[rc].z);

            GEO::index_t vi = locateNearestVertex(p);
            Point3d npp;
            if(vi != GEO::NO_VERTEX)
            {
                npp = _verticesCoords[vi];
            }

            if((vi == GEO::NO_VERTEX) || ((npp - mp->CArr[rc]).size() > minDist))
            {
                vi = _verticesCoords.size();
                _verticesCoords.push_back(p);

                GC_vertexInfo newv;
                newv.nrc = 0;
                newv.segSize = 0;
                newv.segId = -1;

                _camsVertexes[rc] = vi;

                _verticesAttr.push_back(newv);
            }
            else
            {
                _camsVertexes[rc] = vi;
            }
        }
    }
}

void DelaunayGraphCut::addPointsToPreventSingularities(const Point3d voxel[8], float minDist)
{
    ALICEVISION_LOG_DEBUG("Add points to prevent singularities");

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
    for(int i = 0; i < 6; i++)
    {
        Point3d p(extrPts[i].x, extrPts[i].y, extrPts[i].z);
        GEO::index_t vi = locateNearestVertex(p);
        Point3d npp;

        if(vi != GEO::NO_VERTEX)
        {
            npp = _verticesCoords[vi];
        }

        if((vi == GEO::NO_VERTEX) || ((npp - extrPts[i]).size() > minDist))
        {
            _verticesCoords.push_back(p);
            GC_vertexInfo newv;
            newv.nrc = 0;
            newv.segSize = 0;
            newv.segId = -1;

            _verticesAttr.push_back(newv);
        }
    }
}

void DelaunayGraphCut::addHelperPoints(int nGridHelperVolumePointsDim, const Point3d voxel[8], float minDist)
{
    if(nGridHelperVolumePointsDim <= 0)
        return;

    ALICEVISION_LOG_DEBUG("Add helper points");

    int ns = nGridHelperVolumePointsDim;
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

    for(int x = 0; x <= ns; x++)
    {
        for(int y = 0; y <= ns; y++)
        {
            for(int z = 0; z <= ns; z++)
            {
                Point3d pt = voxel[0] + vx * ((float)x / (float)ns) + vy * ((float)y / (float)ns) +
                             vz * ((float)z / (float)ns);
                pt = pt + (CG - pt).normalize() * (maxSize * ((float)rand() / (float)RAND_MAX));

                Point3d p(pt.x, pt.y, pt.z);
                GEO::index_t vi = locateNearestVertex(p);
                Point3d npp;

                if(vi != GEO::NO_VERTEX)
                {
                    npp = _verticesCoords[vi];
                }

                // if there is no nearest vertex or the nearest vertex is not too close
                if((vi == GEO::NO_VERTEX) || ((npp - pt).size() > minDist))
                {
                    _verticesCoords.push_back(p);
                    GC_vertexInfo newv;
                    newv.nrc = 0;
                    newv.segSize = 0;
                    newv.segId = -1;

                    _verticesAttr.push_back(newv);
                }
            }
        }
    }

    ALICEVISION_LOG_DEBUG(" done\n");
}


void DelaunayGraphCut::fuseFromDepthMaps(const StaticVector<int>& cams, const Point3d voxel[8], const FuseParams& params)
{
    ALICEVISION_LOG_INFO("fuseFromDepthMaps, maxVertices: " << params.maxPoints);

    std::vector<Point3d> verticesCoordsPrepare;
    // Load depth from depth maps, select points per depth maps (1 value per tile).
    // Filter points inside other points (with a volume defined by the pixelSize)
    // If too much points at the end, increment a coefficient factor on the pixel size
    // and iterate to fuse points until we get the right amount of points.

    // unsigned long nbValidDepths = computeNumberOfAllPoints(mp, 0);
    // int stepPts = std::ceil((double)nbValidDepths / (double)maxPoints);
    std::size_t nbPixels = 0;
    for(const auto& imgParams: mp->getImagesParams())
    {
        nbPixels += imgParams.size;
    }
    int step = std::floor(std::sqrt(double(nbPixels) / double(params.maxInputPoints)));
    step = std::max(step, params.minStep);
    std::size_t realMaxVertices = 0;
    std::vector<int> startIndex(mp->getNbCameras(), 0);
    for(int i = 0; i < mp->getNbCameras(); ++i)
    {
        const auto& imgParams = mp->getImageParams(i);
        startIndex[i] = realMaxVertices;
        realMaxVertices += std::ceil(imgParams.width / step) * std::ceil(imgParams.height / step);
    }
    verticesCoordsPrepare.resize(realMaxVertices);
    std::vector<double> pixSizePrepare(realMaxVertices);
    std::vector<float> simScorePrepare(realMaxVertices);

    ALICEVISION_LOG_INFO("simFactor: " << params.simFactor);
    ALICEVISION_LOG_INFO("nbPixels: " << nbPixels);
    ALICEVISION_LOG_INFO("maxVertices: " << params.maxPoints);
    ALICEVISION_LOG_INFO("step: " << step);
    ALICEVISION_LOG_INFO("realMaxVertices: " << realMaxVertices);

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
                const std::string depthMapFilepath = getFileNameFromIndex(mp, c, mvsUtils::EFileType::depthMap, 0);
                imageIO::readImage(depthMapFilepath, width, height, depthMap, imageIO::EImageColorSpace::NO_CONVERSION);
                if(depthMap.empty())
                {
                    ALICEVISION_LOG_WARNING("Empty depth map: " << depthMapFilepath);
                    continue;
                }
                int wTmp, hTmp;
                const std::string simMapFilepath = getFileNameFromIndex(mp, c, mvsUtils::EFileType::simMap, 0);
                imageIO::readImage(simMapFilepath, wTmp, hTmp, simMap, imageIO::EImageColorSpace::NO_CONVERSION);
                if(wTmp != width || hTmp != height)
                    throw std::runtime_error("Wrong sim map dimensions: " + simMapFilepath);
                {
                    std::vector<float> simMapTmp(simMap.size());
                    imageIO::convolveImage(width, height, simMap, simMapTmp, "gaussian", params.simGaussianSizeInit, params.simGaussianSizeInit);
                    simMap.swap(simMapTmp);
                }

                const std::string nmodMapFilepath = getFileNameFromIndex(mp, c, mvsUtils::EFileType::nmodMap, 0);
                imageIO::readImage(nmodMapFilepath, wTmp, hTmp, numOfModalsMap, imageIO::EImageColorSpace::NO_CONVERSION);
                if(wTmp != width || hTmp != height)
                    throw std::runtime_error("Wrong nmod map dimensions: " + nmodMapFilepath);
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
                        Point3d p = mp->CArr[c] + (mp->iCamArr[c] * Point2d((float)bestX, (float)bestY)).normalize() * bestDepth;
                        
                        // TODO: isPointInHexahedron: here or in the previous loop per pixel to not loose point?
                        if(voxel == nullptr || mvsUtils::isPointInHexahedron(p, voxel)) 
                        {
                            verticesCoordsPrepare[index] = p;
                            simScorePrepare[index] = bestSimScore;
                            pixSizePrepare[index] = mp->getCamPixelSize(p, c);
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
                                   verticesAttrPrepare, mp, params.simFactor, params.voteMarginFactor, params.contributeMarginFactor, params.simGaussianSize);

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
        if(visCams.size() == 0)
        {
            ALICEVISION_LOG_WARNING("BAD: visCams.size() == 0");
        }
        double maxAngle = 0.0;
        for(int i: visCams)
        {
            for(int j: visCams)
            {
                if(i == j)
                    continue;
                double angle = angleBetwABandAC(verticesCoordsPrepare[vIndex], mp->CArr[i], mp->CArr[j]);
                maxAngle = std::max(angle, maxAngle);
            }
        }
        // Kill the point if the angle is too small
        if(maxAngle < params.minAngleThreshold)
        {
            pixSizePrepare[vIndex] = -1;
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
                                       verticesAttrPrepare, mp, params.simFactor, params.voteMarginFactor, params.contributeMarginFactor, params.simGaussianSize);
    }
    _verticesCoords.swap(verticesCoordsPrepare);
    _verticesAttr.swap(verticesAttrPrepare);

    if(_verticesCoords.size() == 0)
        throw std::runtime_error("Depth map fusion gives an empty result.");

    ALICEVISION_LOG_INFO("fuseFromDepthMaps done: " << _verticesCoords.size() << " points created.");
}

void DelaunayGraphCut::loadPrecomputedDensePoints(const StaticVector<int>* voxelsIds, const Point3d voxel[8], VoxelsGrid* ls)
{
    Point3d cgpt;
    int ncgpt = 0;

    bool doFilterOctreeTracks = mp->userParams.get<bool>("LargeScale.doFilterOctreeTracks", true);
    int minNumOfConsistentCams = mp->userParams.get<int>("filter.minNumOfConsistentCams", 2);

    // add points from voxel
    for(int i = 0; i < voxelsIds->size(); i++)
    {
        ALICEVISION_LOG_INFO("Add points from voxel " << i << " of " << voxelsIds->size() << ".");

        std::string folderName = ls->getVoxelFolderName((*voxelsIds)[i]);
        std::string fileNameTracksPts = folderName + "tracksGridPts.bin";
        std::string fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";

        if(mvsUtils::FileExists(fileNameTracksPts))
        {
            StaticVector<Point3d>* tracksPoints = loadArrayFromFile<Point3d>(fileNameTracksPts);
            StaticVector<StaticVector<Pixel>*>* tracksPointsCams =
                loadArrayOfArraysFromFile<Pixel>(fileNameTracksPtsCams);

            long t1 = mvsUtils::initEstimate();
            for(int j = 0; j < tracksPoints->size(); j++)
            {
                Point3d tp = (*tracksPoints)[j];
                StaticVector<Pixel>* cams = (*tracksPointsCams)[j];
                if((mvsUtils::isPointInHexahedron(tp, voxel)) && (cams != nullptr) &&
                   ((!doFilterOctreeTracks) || ((doFilterOctreeTracks) && (cams->size() >= minNumOfConsistentCams))))
                {
                    cgpt = cgpt + tp;
                    ncgpt++;
                    _verticesCoords.push_back(tp);

                    GC_vertexInfo newv;
                    newv.nrc = 0;
                    newv.segSize = 0;
                    newv.segId = -1;

                    newv.cams.reserve(cams->size());

                    for(int c = 0; c < cams->size(); c++)
                    {
                        int rc = (*cams)[c].x;
                        int nptsrc = (*cams)[c].y;
                        newv.cams.push_back(rc);
                        newv.nrc += nptsrc;
                    }

                    _verticesAttr.push_back(newv);
                }

                mvsUtils::printfEstimate(j, tracksPoints->size(), t1);
            } // for j
            mvsUtils::finishEstimate();

            // delete randIds;

            delete tracksPoints;
            deleteArrayOfArrays<Pixel>(&tracksPointsCams);
        } // if fileexists
    }

    ALICEVISION_LOG_INFO("Dense points loaded.\n");
}


void DelaunayGraphCut::computeVerticesSegSize(bool allPoints, float alpha) // allPoints=true, alpha=0
{
    ALICEVISION_LOG_DEBUG("creating universe");
    int scalePS = mp->userParams.get<int>("global.scalePS", 1);
    int step = mp->userParams.get<int>("global.step", 1);
    float pointToJoinPixSizeDist = (float)mp->userParams.get<double>("delaunaycut.pointToJoinPixSizeDist", 2.0) *
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
        if((v.getNbCameras() > 0) && ((allPoints) || (v.isOnSurface)))
        {
            int rc = v.getCamera(0);

            // go thru all neighbour points
            GEO::vector<VertexIndex> adjVertices;
            _tetrahedralization->get_neighbors(vi, adjVertices);

            for(VertexIndex nvi: adjVertices)
            {
                const GC_vertexInfo& nv = _verticesAttr[nvi];
                const Point3d& np = _verticesCoords[nvi];
                if((vi != nvi) && ((allPoints) || (nv.isOnSurface)))
                {
                    if((p - np).size() <
                       alpha * mp->getCamPixelSize(p, rc)) // TODO FACA: why do we fuse again? And only based on the pixSize of the first camera??
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

    Universe* u = new Universe(_verticesAttr.size());

    // t1 = mvsUtils::initEstimate();
    int s = (int)edges.size(); // Fuse all edges collected to be merged
    for(int i = 0; i < s; i++)
    {
        int a = u->find(edges[i].x);
        int b = u->find(edges[i].y);
        if(a != b) // TODO FACA: Are they not different in all cases?
        {
            u->join(a, b);
        }
        // mvsUtils::printfEstimate(i, s, t1);
    }
    // mvsUtils::finishEstimate();

    // Last loop over vertices to update segId
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        GC_vertexInfo& v = _verticesAttr[vi];
        if(v.isVirtual())
            continue;

        int a = u->find(vi);
        v.segSize = u->elts[a].size;
        v.segId = a;
    }

    delete u;
    ALICEVISION_LOG_DEBUG("creating universe done.");
}

void DelaunayGraphCut::removeSmallSegs(int minSegSize)
{
    ALICEVISION_LOG_DEBUG("removeSmallSegs: " << minSegSize);
    StaticVector<int>* toRemove = new StaticVector<int>();
    toRemove->reserve(getNbVertices());

    for(int i = 0; i < _verticesAttr.size(); ++i)
    {
        const GC_vertexInfo& v = _verticesAttr[i];
        if(v.isReal() && v.segSize < minSegSize)
        {
            toRemove->push_back(i);
        }
    }

    for(int i = 0; i < toRemove->size(); i++)
    {
        int iR = (*toRemove)[i];
        GC_vertexInfo& v = _verticesAttr[iR];

        v.cams.clear();
        // T.remove(fit); // TODO GEOGRAM
    }
    delete toRemove;

    initVertices();
}

bool DelaunayGraphCut::rayCellIntersection(const Point3d& camC, const Point3d& p, int tetrahedron, Facet& out_facet,
                                        bool nearestFarest, Point3d& out_nlpi) const
{
    out_nlpi = p; // important
    out_facet.cellIndex = -1;
    out_facet.localVertexIndex = -1;

    if(isInfiniteCell(tetrahedron))
    {
        return false;
    }

    Point3d linePoint = p;
    Point3d lineVect = (camC - p).normalize();
    double mind = (camC - p).size();

    // go thru all triangles
    bool existsTriOnRay = false;
    const Point3d* A = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 0)]);
    const Point3d* B = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 1)]);
    const Point3d* C = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 2)]);
    const Point3d* D = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 3)]);

    // All the facets of the tetrahedron
    std::array<std::array<const Point3d*, 3>, 4> facets {{
        {B, C, D}, // opposite vertex A, index 0
        {A, C, D}, // opposite vertex B, index 1
        {A, B, D}, // opposite vertex C, index 2
        {A, B, C}  // opposite vertex D, index 3
    }};
    int oppositeVertexIndex = -1;
    Point3d lpi;
    double dist;
    // Test all facets of the tetrahedron
    for(int i = 0; i < 4; ++i)
    {
        const auto& facet= facets[i];
        if(isLineInTriangle(&lpi, facet[0], facet[1], facet[2], &linePoint, &lineVect))
        {
            dist = (camC - lpi).size();
            if(nearestFarest)
            {
                if(dist < mind) // between the camera and the point
                {
                    oppositeVertexIndex = i;
                    existsTriOnRay = true;
                    mind = dist;
                    out_nlpi = lpi;
                    //break;
                }
            }
            else
            {
                if(dist > mind) // behind the point (from the camera)
                {
                    oppositeVertexIndex = i;
                    existsTriOnRay = true;
                    mind = dist;
                    out_nlpi = lpi;
                    //break;
                }
            }
        }
    }

    if(!existsTriOnRay)
        return false;

    out_facet.cellIndex = tetrahedron; // cell handle
    out_facet.localVertexIndex = oppositeVertexIndex; // vertex index that is not in the intersected facet
    return true;
}

DelaunayGraphCut::Facet DelaunayGraphCut::getFacetInFrontVertexOnTheRayToThePoint3d(VertexIndex vi,
                                                                                   Point3d& ptt) const
{
    const Point3d& p = _verticesCoords[vi];

    double minDist = (ptt - p).size(); // initialize minDist to the distance from 3d point p to camera center c
    Facet nearestFacet;
    nearestFacet.cellIndex = -1;
    nearestFacet.localVertexIndex = -1;

    for(int k = 0; true; ++k)
    {
        CellIndex adjCellIndex = vertexToCells(vi, k); // GEOGRAM: set_stores_cicl(true) required
        if(adjCellIndex == GEO::NO_CELL) // last one
            break;

        if(isInfiniteCell(adjCellIndex))
            continue;
        Facet f1;
        Point3d lpi;
        if(rayCellIntersection(ptt, p, adjCellIndex, f1, true, lpi) == true)
        {
            // if it is inbetween the camera and the point
            if((ptt - lpi).size() < minDist)
            {
                nearestFacet = f1;
                minDist = (ptt - lpi).size();
            }
            // TODO FACA: maybe we can break and remove minDist?
        }
    }
    return nearestFacet;
}

DelaunayGraphCut::Facet DelaunayGraphCut::getFacetBehindVertexOnTheRayToTheCam(VertexIndex vi,
                                                                              int cam) const
{
    const Point3d& p = _verticesCoords[vi];

    double maxDist = (mp->CArr[cam] - p).size();
    Facet farestFacet;

    for(int k = 0; true; ++k)
    {
        CellIndex adjCellIndex = vertexToCells(vi, k); // GEOGRAM: set_stores_cicl(true) required
        if(adjCellIndex == GEO::NO_CELL) // last one
            break;
        if(isInfiniteCell(adjCellIndex))
            continue;

        Facet f1;
        Point3d lpi;
        if(rayCellIntersection(mp->CArr[cam], p, adjCellIndex, f1, false, lpi) == true)
        {
            // if it is after the point p (along the axis from the camera)
            if((mp->CArr[cam] - lpi).size() > maxDist)
            {
                farestFacet = f1;
                maxDist = (mp->CArr[cam] - lpi).size();
            }
        }
    }
    return farestFacet;
}

int DelaunayGraphCut::getFirstCellOnTheRayFromCamToThePoint(int cam, Point3d& p, Point3d& lpi) const
{
    int cam_vi = _camsVertexes[cam];
    Point3d camBehind = mp->CArr[cam] + (mp->CArr[cam] - p);
    Point3d dir = (p - mp->CArr[cam]).normalize();
    float maxdist = 0.0f;
    int farestCell = -1;

    for(int k = 0; true; ++k)
    {
        CellIndex adjCellIndex = vertexToCells(cam_vi, k); // GEOGRAM: set_stores_cicl(true) required
        if(adjCellIndex == GEO::NO_CELL) // last one
            break;
        if(isInfiniteCell(adjCellIndex))
            continue;

        Facet f1;
        if(rayCellIntersection(camBehind, mp->CArr[cam], adjCellIndex, f1, false, lpi) == true)
        {
            float dist = orientedPointPlaneDistance(lpi, camBehind, dir);
            if(dist > maxdist)
            {
                maxdist = dist;
                farestCell = adjCellIndex;
            }
        }
    }

    return farestCell;
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

void DelaunayGraphCut::fillGraph(bool fixesSigma, float nPixelSizeBehind, bool allPoints, bool behind,
                               bool labatutWeights, bool fillOut, float distFcnHeight) // fixesSigma=true nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 labatutWeights=0 fillOut=1 distFcnHeight=0
{
    ALICEVISION_LOG_INFO("Computing s-t graph weights.");
    long t1 = clock();

    setIsOnSurface();

    // loop over all cells ... initialize
    for(GC_cellInfo& c: _cellsAttr)
    {
        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.in = 0.0f;
        c.out = 0.0f;
        c.on = 0.0f;
        for(int s = 0; s < 4; s++)
        {
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }

    // choose random order to prevent waiting
    StaticVector<int>* vetexesToProcessIdsRand = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size());

    int64_t avStepsFront = 0;
    int64_t aAvStepsFront = 0;
    int64_t avStepsBehind = 0;
    int64_t nAvStepsBehind = 0;
    int avCams = 0;
    int nAvCams = 0;

#pragma omp parallel for reduction(+:avStepsFront,aAvStepsFront,avStepsBehind,nAvStepsBehind,avCams,nAvCams)
    for(int i = 0; i < vetexesToProcessIdsRand->size(); i++)
    {
        int iV = (*vetexesToProcessIdsRand)[i];
        const GC_vertexInfo& v = _verticesAttr[iV];

        if(v.isReal() && (allPoints || v.isOnSurface) && (v.nrc > 0))
        {
            for(int c = 0; c < v.cams.size(); c++)
            {
                // "weight" is called alpha(p) in the paper
                float weight = weightFcn((float)v.nrc, labatutWeights, v.getNbCameras()); // number of cameras

                assert(v.cams[c] >= 0);
                assert(v.cams[c] < mp->ncams);

                int nstepsFront = 0;
                int nstepsBehind = 0;
                fillGraphPartPtRc(nstepsFront, nstepsBehind, iV, v.cams[c], weight, fixesSigma, nPixelSizeBehind,
                                  allPoints, behind, fillOut, distFcnHeight);

                avStepsFront += nstepsFront;
                aAvStepsFront += 1;
                avStepsBehind += nstepsBehind;
                nAvStepsBehind += 1;
            } // for c

            avCams += v.cams.size();
            nAvCams += 1;
        }
    }

    delete vetexesToProcessIdsRand;

    ALICEVISION_LOG_DEBUG("avStepsFront " << avStepsFront);
    ALICEVISION_LOG_DEBUG("avStepsFront = " << mvsUtils::num2str(avStepsFront) << " // " << mvsUtils::num2str(aAvStepsFront));
    ALICEVISION_LOG_DEBUG("avStepsBehind = " << mvsUtils::num2str(avStepsBehind) << " // " << mvsUtils::num2str(nAvStepsBehind));
    ALICEVISION_LOG_DEBUG("avCams = " << mvsUtils::num2str(avCams) << " // " << mvsUtils::num2str(nAvCams));

    mvsUtils::printfElapsedTime(t1, "s-t graph weights computed : ");
}

void DelaunayGraphCut::fillGraphPartPtRc(int& out_nstepsFront, int& out_nstepsBehind, int vertexIndex, int cam,
                                       float weight, bool fixesSigma, float nPixelSizeBehind, bool allPoints,
                                       bool behind, bool fillOut, float distFcnHeight)  // fixesSigma=true nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 fillOut=1 distFcnHeight=0
{
    out_nstepsFront = 0;
    out_nstepsBehind = 0;

    int maxint = 1000000; // std::numeric_limits<int>::std::max()

    const Point3d& po = _verticesCoords[vertexIndex];
    const float pixSize = mp->getCamPixelSize(po, cam);
    float maxDist = nPixelSizeBehind * pixSize;
    if(fixesSigma)
    {
        maxDist = nPixelSizeBehind;
    }

    assert(cam >= 0);
    assert(cam < mp->ncams);

    // printf("%f %f %f\n",po.x,po.y,po.z);

    int nsteps = 0;

    if(fillOut)
    {
        out_nstepsFront = 0;
        // tetrahedron connected to the point p and which intersect the ray from camera c to point p
        CellIndex ci = getFacetInFrontVertexOnTheRayToTheCam(vertexIndex, cam).cellIndex;

        Point3d p = po;
        CellIndex lastFinite = GEO::NO_CELL;
        bool ok = ci != GEO::NO_CELL;
        while(ok)
        {
            {
#pragma OMP_ATOMIC_UPDATE
                _cellsAttr[ci].out += weight;
            }

            ++out_nstepsFront;
            ++nsteps;

            Point3d pold = p;
            Facet f1, f2;
            Point3d lpi;
            // find cell which is nearest to the cam and which is intersected with cam-p ray
            if(!nearestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, ci, f1, f2, lpi))
            {
                ok = false;
            }
            else
            {
                float dist = distFcn(maxDist, (po - pold).size(), distFcnHeight);

                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[f1.cellIndex].gEdgeVisWeight[f1.localVertexIndex] += weight * dist;
                }

                if(f2.cellIndex == GEO::NO_CELL)
                    ok = false;
                ci = f2.cellIndex;
                lastFinite = f2.cellIndex;
            }
        }

        // get the outer tetrahedron of camera c for the ray to p = the last tetrahedron
        if(lastFinite != GEO::NO_CELL)
        {
#pragma OMP_ATOMIC_WRITE
            _cellsAttr[lastFinite].cellSWeight = (float)maxint;
        }
    }

    {
        out_nstepsBehind = 0;
        // get the tetrahedron next to point p on the ray from c
        Facet f1 = getFacetBehindVertexOnTheRayToTheCam(vertexIndex, cam);
        Facet f2;

        CellIndex ci = f1.cellIndex;
        if(ci != GEO::NO_CELL)
        {
#pragma OMP_ATOMIC_UPDATE
            _cellsAttr[ci].on += weight;
        }

        Point3d p = po; // HAS TO BE HERE !!!

        bool ok = (ci != GEO::NO_CELL) && allPoints;
        while(ok)
        {
            GC_cellInfo& c = _cellsAttr[ci];
            {
                if(behind)
                {
#pragma OMP_ATOMIC_UPDATE
                    c.cellTWeight += weight;
                }
#pragma OMP_ATOMIC_UPDATE
                c.in += weight;
            }

            ++out_nstepsBehind;
            ++nsteps;

            Point3d pold = p;
            Point3d lpi;
            // find cell which is farest to the cam and which intersect cam-p ray
            if((!farestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, ci, f1, f2, lpi)) ||
               ((po - pold).size() >= maxDist) || (!allPoints))
            {
                ok = false;
            }
            else
            {
                // float dist = 1.0f;
                // float dist = distFcn(maxDist,(po-pold).size()); // not sure if it is OK ... TODO
                // check ... with, without, and so on ....
                // because labatutCFG09 with 32 gives much better result than nrc
                // but when using just nrc and not using distFcn then the result is the same as labatutCGF09

                float dist = distFcn(maxDist, (po - pold).size(), distFcnHeight);

                if(f2.cellIndex == GEO::NO_CELL)
                {
                    ok = false;
                }
                else
                {
#pragma OMP_ATOMIC_UPDATE
                    _cellsAttr[f2.cellIndex].gEdgeVisWeight[f2.localVertexIndex] += weight * dist;
                }
                ci = f2.cellIndex;
            }
        }

        // cv: is the tetrahedron in distance 2*sigma behind the point p in the direction of the camera c (called Lcp in the paper)
        if(!behind)
        {
            if(ci != GEO::NO_CELL)
            {
#pragma OMP_ATOMIC_UPDATE
                _cellsAttr[ci].cellTWeight += weight;
            }
        }
    }
}

void DelaunayGraphCut::forceTedgesByGradientCVPR11(bool fixesSigma, float nPixelSizeBehind)
{
    ALICEVISION_LOG_INFO("Forcing t-edges.");
    long t2 = clock();

    float delta = (float)mp->userParams.get<double>("delaunaycut.delta", 0.1f);
    ALICEVISION_LOG_INFO("delta: " << delta);

    float beta = (float)mp->userParams.get<double>("delaunaycut.beta", 1000.0f);
    ALICEVISION_LOG_INFO("beta: " << beta);

    for(GC_cellInfo& c: _cellsAttr)
    {
        c.on = 0.0f;
    }

    // choose random order to prevent waiting
    StaticVector<int>* vetexesToProcessIdsRand = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size());

#pragma omp parallel for
    for(int i = 0; i < vetexesToProcessIdsRand->size(); ++i)
    {
        int vi = (*vetexesToProcessIdsRand)[i];
        GC_vertexInfo& v = _verticesAttr[vi];
        if(v.isVirtual())
            continue;

        const Point3d& po = _verticesCoords[vi];
        for(int c = 0; c < v.cams.size(); ++c)
        {
            int cam = v.cams[c];

            Facet fFirst = getFacetInFrontVertexOnTheRayToTheCam(vi, cam);

            // get the tetrahedron next to point p on the ray from c
            Facet f2;
            Facet f1 = getFacetBehindVertexOnTheRayToTheCam(vi, cam);

            if((fFirst.cellIndex != GEO::NO_CELL) && (f1.cellIndex != GEO::NO_CELL) && (!isInfiniteCell(f1.cellIndex)))
            {
                float eFirst = _cellsAttr[fFirst.cellIndex].out;

                f2 = mirrorFacet(f1);
                CellIndex ci = f1.cellIndex;
                Point3d p = po; // HAS TO BE HERE !!!
                float maxDist = nPixelSizeBehind * mp->getCamPixelSize(p, cam);
                if(fixesSigma)
                {
                    maxDist = nPixelSizeBehind;
                }

                bool ok = (ci != GEO::NO_CELL);
                while(ok)
                {
                    Point3d pold = p;
                    Point3d lpi;
                    Facet ff1, ff2;
                    // find cell which is farest to the cam and which is
                    // intersected with cam-p
                    // ray
                    if((!farestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, ci, ff1, ff2, lpi)) ||
                       ((po - pold).size() >= maxDist))
                    {
                        ok = false;
                    }
                    else
                    {
                        if(ff2.cellIndex == GEO::NO_CELL)
                            ok = false;
                        ci = ff2.cellIndex;
                        f1 = ff1;
                        f2 = ff2;
                    }
                }

                if(ci != GEO::NO_CELL)
                {
                    float eLast = _cellsAttr[f2.cellIndex].out;
                    if((eFirst > eLast) && (eFirst < beta) && (eLast / eFirst < delta))
                    {
#pragma OMP_ATOMIC_UPDATE
                        _cellsAttr[ci].on += (eFirst - eLast);
                    }
                }
            }

        } // for c

    } // for i

    delete vetexesToProcessIdsRand;

    for(GC_cellInfo& c: _cellsAttr)
    {
        c.cellTWeight = std::max(c.cellTWeight, std::min(1000000.0f, std::max(1.0f, c.cellTWeight) * c.on));
    }

    mvsUtils::printfElapsedTime(t2, "t-edges forced: ");
}

void DelaunayGraphCut::forceTedgesByGradientIJCV(bool fixesSigma, float nPixelSizeBehind)
{
    ALICEVISION_LOG_INFO("Forcing t-edges");
    long t2 = clock();

    float delta = (float)mp->userParams.get<double>("delaunaycut.delta", 0.1f);
    ALICEVISION_LOG_DEBUG("delta: " << delta);

    float minJumpPartRange = (float)mp->userParams.get<double>("delaunaycut.minJumpPartRange", 10000.0f);
    ALICEVISION_LOG_DEBUG("minJumpPartRange: " << minJumpPartRange);

    float maxSilentPartRange = (float)mp->userParams.get<double>("delaunaycut.maxSilentPartRange", 100.0f);
    ALICEVISION_LOG_DEBUG("maxSilentPartRange: " << maxSilentPartRange);

    float nsigmaJumpPart = (float)mp->userParams.get<double>("delaunaycut.nsigmaJumpPart", 2.0f);
    ALICEVISION_LOG_DEBUG("nsigmaJumpPart: " << nsigmaJumpPart);

    float nsigmaFrontSilentPart = (float)mp->userParams.get<double>("delaunaycut.nsigmaFrontSilentPart", 2.0f);
    ALICEVISION_LOG_DEBUG("nsigmaFrontSilentPart: " << nsigmaFrontSilentPart);

    float nsigmaBackSilentPart = (float)mp->userParams.get<double>("delaunaycut.nsigmaBackSilentPart", 2.0f);
    ALICEVISION_LOG_DEBUG("nsigmaBackSilentPart: " << nsigmaBackSilentPart);


    for(GC_cellInfo& c: _cellsAttr)
    {
        c.on = 0.0f;
        // WARNING out is not the same as the sum because the sum are counted edges behind as well
        // c.out = c.gEdgeVisWeight[0] + c.gEdgeVisWeight[1] + c.gEdgeVisWeight[2] + c.gEdgeVisWeight[3];
    }

    // choose random order to prevent waiting
    StaticVector<int>* vetexesToProcessIdsRand = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size());

    int64_t avStepsFront = 0;
    int64_t aAvStepsFront = 0;
    int64_t avStepsBehind = 0;
    int64_t nAvStepsBehind = 0;

#pragma omp parallel for reduction(+:avStepsFront,aAvStepsFront,avStepsBehind,nAvStepsBehind)
    for(int i = 0; i < vetexesToProcessIdsRand->size(); ++i)
    {
        int vi = (*vetexesToProcessIdsRand)[i];
        GC_vertexInfo& v = _verticesAttr[vi];
        if(v.isVirtual())
            continue;

        const Point3d& po = _verticesCoords[vi];
        for(int c = 0; c < v.cams.size(); ++c)
        {
            int nstepsFront = 0;
            int nstepsBehind = 0;

            int cam = v.cams[c];
            float maxDist = 0.0f;
            if(fixesSigma)
            {
                maxDist = nPixelSizeBehind;
            }
            else
            {
                maxDist = nPixelSizeBehind * mp->getCamPixelSize(po, cam);
            }

            float minJump = 10000000.0f;
            float minSilent = 10000000.0f;
            float maxJump = 0.0f;
            float maxSilent = 0.0f;
            float midSilent = 10000000.0f;

            {
                CellIndex ci = getFacetInFrontVertexOnTheRayToTheCam(vi, cam).cellIndex;
                Point3d p = po; // HAS TO BE HERE !!!
                bool ok = (ci != GEO::NO_CELL);
                while(ok)
                {
                    ++nstepsFront;

                    const GC_cellInfo& c = _cellsAttr[ci];
                    if((p - po).size() > nsigmaFrontSilentPart * maxDist) // (p-po).size() > 2 * sigma
                    {
                        minJump = std::min(minJump, c.out);
                        maxJump = std::max(maxJump, c.out);
                    }
                    else
                    {
                        minSilent = std::min(minSilent, c.out);
                        maxSilent = std::max(maxSilent, c.out);
                    }

                    Facet f1, f2;
                    Point3d lpi;
                    // find cell which is nearest to the cam and which intersect cam-p ray
                    if(((p - po).size() > (nsigmaJumpPart + nsigmaFrontSilentPart) * maxDist) || // (2 + 2) * sigma
                       (!nearestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, ci, f1, f2, lpi)))
                    {
                        ok = false;
                    }
                    else
                    {
                        if(f2.cellIndex == GEO::NO_CELL)
                            ok = false;
                        ci = f2.cellIndex;
                    }
                }
            }

            {
                CellIndex ci = getFacetBehindVertexOnTheRayToTheCam(vi, cam).cellIndex; // T1
                Point3d p = po; // HAS TO BE HERE !!!
                bool ok = (ci != GEO::NO_CELL);
                if(ok)
                {
                    midSilent = _cellsAttr[ci].out;
                }

                while(ok)
                {
                    nstepsBehind++;
                    const GC_cellInfo& c = _cellsAttr[ci];

                    minSilent = std::min(minSilent, c.out);
                    maxSilent = std::max(maxSilent, c.out);

                    Facet f1, f2;
                    Point3d lpi;
                    // find cell which is farest to the cam and which intersect cam-p ray
                    if(((p - po).size() > nsigmaBackSilentPart * maxDist) || // (p-po).size() > 2 * sigma
                       (!farestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, ci, f1, f2, lpi)))
                    {
                        ok = false;
                    }
                    else
                    {
                        if(f2.cellIndex == GEO::NO_CELL)
                            ok = false;
                        ci = f2.cellIndex;
                    }
                }

                if(ci != GEO::NO_CELL)
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

                    if(
                       (midSilent / maxJump < delta) && // (g / B) < k_rel              //// k_rel=0.1
                       (maxJump - midSilent > minJumpPartRange) && // (B - g) > k_abs   //// k_abs=10000 // 1000 in the paper
                       (maxSilent < maxSilentPartRange)) // g < k_outl                  //// k_outl=100  // 400 in the paper
                        //(maxSilent-minSilent<maxSilentPartRange))
                    {
#pragma OMP_ATOMIC_UPDATE
                        _cellsAttr[ci].on += (maxJump - midSilent);
                    }
                }
            }

            avStepsFront += nstepsFront;
            aAvStepsFront += 1;
            avStepsBehind += nstepsBehind;
            nAvStepsBehind += 1;
        }
    }

    delete vetexesToProcessIdsRand;

    for(GC_cellInfo& c: _cellsAttr)
    {
        float w = std::max(1.0f, c.cellTWeight);

        // cellTWeight = clamp(w * c.on, cellTWeight, 1000000.0f);
        c.cellTWeight = std::max(c.cellTWeight, std::min(1000000.0f, w * c.on));
        // c.cellTWeight = std::max(c.cellTWeight,fit->info().on);
    }

    {
        ALICEVISION_LOG_DEBUG("avStepsFront = " << avStepsFront << " // " << aAvStepsFront);
        ALICEVISION_LOG_DEBUG("avStepsBehind = " << avStepsBehind << " // " << nAvStepsBehind);
    }
    mvsUtils::printfElapsedTime(t2, "t-edges forced: ");
}

void DelaunayGraphCut::updateGraphFromTmpPtsCamsHexah(const StaticVector<int>& incams, Point3d hexah[8],
                                                    std::string tmpCamsPtsFolderName, bool labatutWeights,
                                                    float distFcnHeight)
{
    ALICEVISION_LOG_INFO("Updating: LSC.");

#pragma omp parallel for
    for(int c = 0; c < incams.size(); c++)
    {
        int rc = incams[c];
        std::string camPtsFileName;
        camPtsFileName = tmpCamsPtsFolderName + "camPtsGrid_" + mvsUtils::num2strFourDecimal(rc) + ".bin";
        ALICEVISION_LOG_INFO("File exist? " << camPtsFileName);
        if(mvsUtils::FileExists(camPtsFileName))
        {
            updateGraphFromTmpPtsCamsHexahRC(rc, hexah, tmpCamsPtsFolderName, labatutWeights, distFcnHeight);
        }
    } // for c
}

void DelaunayGraphCut::updateGraphFromTmpPtsCamsHexahRC(int rc, Point3d hexah[8], std::string tmpCamsPtsFolderName,
                                                      bool labatutWeights, float  /*distFcnHeight*/)
{
    ALICEVISION_LOG_INFO("DelaunayGraphCut::updateGraphFromTmpPtsCamsHexahRC: rc: " << rc);

    // fill edges
    int nin = 0;
    int nout = 0;
    int cnull = 0;
    int nwup = 0;

    std::string camPtsFileName;
    camPtsFileName = tmpCamsPtsFolderName + "camPtsGrid_" + mvsUtils::num2strFourDecimal(rc) + ".bin";

    bool doFilterOctreeTracks = mp->userParams.get<bool>("LargeScale.doFilterOctreeTracks", true);
    int minNumOfConsistentCams = mp->userParams.get<int>("filter.minNumOfConsistentCams", 2);

    /////////////////////////////////////////////////////////////////////////////////
    FILE* f = fopen(camPtsFileName.c_str(), "rb");
    while(feof(f) == 0)
    {
        GC_camVertexInfo pnt;
        pnt.freadinfo(f);
        // printf("%f, %i, %f, %f,
        // %f\n",pnt.sim,pnt.nrc,pnt.point.x,pnt.point.y,pnt.point.z);
        Point3d pt = pnt.point;

        float weight = weightFcn((float)pnt.nrc, labatutWeights, (float)pnt.ncams);

        nout++;

        StaticVector<Point3d>* lshi = mvsUtils::lineSegmentHexahedronIntersection(pt, mp->CArr[rc], hexah);
        if((!mvsUtils::isPointInHexahedron(pt, hexah)) && (lshi->size() >= 1) && (feof(f) == 0) &&
           ((!doFilterOctreeTracks) || ((doFilterOctreeTracks) && (pnt.ncams >= minNumOfConsistentCams))))
        {
            nin++;
            Point3d lpi = pt;
            Point3d camBehind = mp->CArr[rc] + (mp->CArr[rc] - pt);
            CellIndex ci = getFirstCellOnTheRayFromCamToThePoint(rc, pt, lpi);
            if(ci != GEO::NO_CELL)
            {
                // update weights on the sp-cam half line
                CellIndex tmp_ci = ci;
                Point3d p = mp->CArr[rc];
                // _cellsAttr[ci].cellSWeight = (float)maxint;

                bool ok = tmp_ci != GEO::NO_CELL;

                while(ok)
                {
                    {
#pragma OMP_ATOMIC_UPDATE
                        _cellsAttr[tmp_ci].out += weight;
                    }

                    Facet f1, f2;
                    Point3d lpi;
                    // find cell which is nearest to the pont and which is
                    // intersected with point-p ray
                    if(!farestNeighCellToTheCamOnTheRay(camBehind, p, tmp_ci, f1, f2, lpi))
                    {
                        ok = false;
                    }
                    else
                    {
                        if(f2.cellIndex == GEO::NO_CELL)
                        {
                            ok = false;
                        }
                        else
                        {
#pragma OMP_ATOMIC_UPDATE
                            _cellsAttr[f2.cellIndex].gEdgeVisWeight[f2.localVertexIndex] += weight;
                        }
                        tmp_ci = f2.cellIndex;
                        ++nwup;
                    }
                }
            }
            else
            {
                ++cnull;
            }
        }
        delete lshi;
    }
    fclose(f);

    // ALICEVISION_LOG_DEBUG("in: " << nin, ", cnull:" << cnull << " nwup: " << nwup << ", out: " << nout);
}

int DelaunayGraphCut::setIsOnSurface()
{
    // set is on surface
    for(GC_vertexInfo& v: _verticesAttr)
    {
        v.isOnSurface = false;
    }

    int nbSurfaceFacets = 0;
    // loop over all facets
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellIsFull[f1.cellIndex]; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellIsFull[f2.cellIndex]; // get if it is occupied

            if(uo == vo)
                continue;

            VertexIndex v1 = getVertexIndex(f1, 0);
            VertexIndex v2 = getVertexIndex(f1, 1);
            VertexIndex v3 = getVertexIndex(f1, 2);
            ++nbSurfaceFacets;
            _verticesAttr[v1].isOnSurface = true;
            _verticesAttr[v2].isOnSurface = true;
            _verticesAttr[v3].isOnSurface = true;

            assert(!(isInfiniteCell(f1.cellIndex) && isInfiniteCell(f2.cellIndex))); // infinite both cells of finite vertex!
        }
    }
    ALICEVISION_LOG_INFO("setIsOnSurface nbSurfaceFacets: " << nbSurfaceFacets);
    return nbSurfaceFacets;
}

void DelaunayGraphCut::graphCutPostProcessing()
{
    long timer = std::clock();
    ALICEVISION_LOG_INFO("Graph cut post-processing.");
    invertFullStatusForSmallLabels();

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
    for(std::size_t i = 0; i < toDoInverse.size(); ++i)
    {
        CellIndex ci = toDoInverse[i];
        _cellIsFull[ci] = !_cellIsFull[ci];
    }
    ALICEVISION_LOG_INFO("Graph cut post-processing done.");

    mvsUtils::printfElapsedTime(timer, "Graph cut post-processing ");
}

void DelaunayGraphCut::freeUnwantedFullCells(const Point3d* hexah)
{
    ALICEVISION_LOG_DEBUG("freeUnwantedFullCells\n");

    int minSegmentSize = (int)mp->userParams.get<int>("hallucinationsFiltering.minSegmentSize", 10);
    bool doRemoveBubbles = (bool)mp->userParams.get<bool>("hallucinationsFiltering.doRemoveBubbles", true);
    bool doRemoveDust = (bool)mp->userParams.get<bool>("hallucinationsFiltering.doRemoveDust", true);
    bool doLeaveLargestFullSegmentOnly = (bool)mp->userParams.get<bool>("hallucinationsFiltering.doLeaveLargestFullSegmentOnly", false);

    if(doRemoveBubbles)
    {
        // Remove empty spaces that are not connected to at least one camera
        removeBubbles();
    }

    // free all full cell that have a camera vertex
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        VertexIndex cam_vi = _camsVertexes[rc];
        if(cam_vi == GEO::NO_VERTEX)
            continue;
        for(int k = 0; true; ++k)
        {
            CellIndex adjCellIndex = vertexToCells(cam_vi, k); // GEOGRAM: set_stores_cicl(true) required
            if(adjCellIndex == GEO::NO_CELL) // last one
                break;
            if(isInfiniteCell(adjCellIndex))
                continue;

            _cellIsFull[adjCellIndex] = false;
        }
    }

    // remove cells that have a point outside hexahedron
    if(hexah != nullptr)
    {
        int nremoved = 0;
        Point3d hexahinf[8];
        mvsUtils::inflateHexahedron(hexah, hexahinf, 1.001);
        for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
        {
            if(isInfiniteCell(ci) || !_cellIsFull[ci])
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
                ++nremoved;
            }
        }
        ALICEVISION_LOG_DEBUG(nremoved << " removed cells outside hexahedron");
    }

    if(doRemoveDust)
    {
        removeDust(minSegmentSize);
    }

    if(doLeaveLargestFullSegmentOnly)
    {
        leaveLargestFullSegmentOnly();
    }
}

void DelaunayGraphCut::invertFullStatusForSmallLabels()
{
    ALICEVISION_LOG_DEBUG("filling small holes");

    const std::size_t nbCells = _cellIsFull.size();
    StaticVector<int>* colorPerCell = new StaticVector<int>();
    colorPerCell->reserve(nbCells);
    colorPerCell->resize_with(nbCells, -1);

    StaticVector<int>* nbCellsPerColor = new StaticVector<int>();
    nbCellsPerColor->reserve(100);
    nbCellsPerColor->resize_with(1, 0);
    int lastColorId = 0;

    StaticVector<CellIndex>* buff = new StaticVector<CellIndex>();
    buff->reserve(nbCells);

    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        if((*colorPerCell)[ci] == -1)
        {
            // backtrack all connected interior cells
            buff->resize(0);
            buff->push_back(ci);

            (*colorPerCell)[ci] = lastColorId;
            (*nbCellsPerColor)[lastColorId] += 1;

            while(buff->size() > 0)
            {
                CellIndex tmp_ci = buff->pop();

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    if(((*colorPerCell)[nci] == -1) && (_cellIsFull[nci] == _cellIsFull[ci]))
                    {
                        (*colorPerCell)[nci] = lastColorId;
                        (*nbCellsPerColor)[lastColorId] += 1;
                        buff->push_back(nci);
                    }
                }
            }
            nbCellsPerColor->push_back(0); // add new color with 0 cell
            ++lastColorId;
            assert(lastColorId == nbCellsPerColor->size() - 1);
        }
    }
    assert((*nbCellsPerColor)[nbCellsPerColor->size()-1] == 0);
    nbCellsPerColor->resize(nbCellsPerColor->size()-1); // remove last empty element

    int nfilled = 0;
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        if((*nbCellsPerColor)[(*colorPerCell)[ci]] < 100)
        {
            _cellIsFull[ci] = !_cellIsFull[ci];
            ++nfilled;
        }
    }

    ALICEVISION_LOG_DEBUG("Full number of cells: " << nbCells << ", Number of labels: " << nbCellsPerColor->size() << ", Number of cells changed: " << nfilled);

    delete nbCellsPerColor;
    delete colorPerCell;
    delete buff;
}

void DelaunayGraphCut::createDensePointCloudFromPrecomputedDensePoints(Point3d hexah[8], const StaticVector<int>& cams, StaticVector<int>* voxelsIds, VoxelsGrid* ls)
{
  // Load tracks
  ALICEVISION_LOG_INFO("Creating delaunay tetrahedralization from depth maps voxel");

  float minDist = hexah ? (hexah[0] - hexah[1]).size() / 1000.0f : 0.00001f;

  // add points for cam centers
  addPointsFromCameraCenters(cams, minDist);

  // add 6 points to prevent singularities
  addPointsToPreventSingularities(hexah, minDist);

  loadPrecomputedDensePoints(voxelsIds, hexah, ls);

  // initialize random seed
  srand(time(nullptr));

  int nGridHelperVolumePointsDim = mp->userParams.get<int>("LargeScale.nGridHelperVolumePointsDim", 10);

  // add volume points to prevent singularities
  addHelperPoints(nGridHelperVolumePointsDim, hexah, minDist);
}


void DelaunayGraphCut::createDensePointCloud(Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData* sfmData, const FuseParams* depthMapsFuseParams)
{
  assert(sfmData != nullptr || depthMapsFuseParams != nullptr);

  ALICEVISION_LOG_INFO("Creating dense point cloud.");

  float minDist = hexah ? (hexah[0] - hexah[1]).size() / 1000.0f : 0.00001f;

  // add points for cam centers
  addPointsFromCameraCenters(cams, minDist);

  // add 6 points to prevent singularities
  addPointsToPreventSingularities(hexah, minDist);

  // add points from depth maps
  if(depthMapsFuseParams != nullptr)
    fuseFromDepthMaps(cams, hexah, *depthMapsFuseParams);

  // add points from sfm
  if(sfmData != nullptr)
    addPointsFromSfM(hexah, cams, *sfmData);

  // initialize random seed
  srand(time(nullptr));

  const int nGridHelperVolumePointsDim = mp->userParams.get<int>("LargeScale.nGridHelperVolumePointsDim", 10);

  // add volume points to prevent singularities
  addHelperPoints(nGridHelperVolumePointsDim, hexah, minDist);
}

void DelaunayGraphCut::createGraphCut(Point3d hexah[8], const StaticVector<int>& cams, VoxelsGrid* ls, const std::string& folderName, const std::string& tmpCamsPtsFolderName, bool removeSmallSegments, const Point3d& spaceSteps)
{
  initVertices();

  // Create tetrahedralization
  computeDelaunay();
  displayStatistics();

  computeVerticesSegSize(true, 0.0f); // TODO: could go into the "if(removeSmallSegments)"?

  if(removeSmallSegments) // false
    removeSmallSegs(2500); // TODO FACA: to decide

  const bool updateLSC = ls ? mp->userParams.get<bool>("LargeScale.updateLSC", true) : false;

  reconstructExpetiments(cams, folderName, updateLSC,
                         hexah, tmpCamsPtsFolderName,
                         spaceSteps);

  if(mp->userParams.get<bool>("LargeScale.saveDelaunayTriangulation", false))
  {
    const std::string fileNameDh = folderName + "delaunayTriangulation.bin";
    const std::string fileNameInfo = folderName + "delaunayTriangulationInfo.bin";
    saveDh(fileNameDh, fileNameInfo);
  }
}

void DelaunayGraphCut::addToInfiniteSw(float sW)
{
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        if(isInfiniteCell(ci))
        {
            GC_cellInfo& c = _cellsAttr[ci];
            c.cellSWeight += sW;
        }
    }
}

void DelaunayGraphCut::reconstructGC(const Point3d* hexah)
{
    ALICEVISION_LOG_INFO("reconstructGC start.");

    maxflow();

    ALICEVISION_LOG_INFO("Maxflow: convert result to surface.");
    // Convert cells FULL/EMPTY into surface
    setIsOnSurface();

    freeUnwantedFullCells(hexah);

    ALICEVISION_LOG_INFO("reconstructGC done.");
}

void DelaunayGraphCut::maxflow()
{
    long t_maxflow = clock();

    ALICEVISION_LOG_INFO("Maxflow: start allocation.");
    // MaxFlow_CSR maxFlowGraph(_cellsAttr.size());
    MaxFlow_AdjList maxFlowGraph(_cellsAttr.size());

    ALICEVISION_LOG_INFO("Maxflow: add nodes.");
    // fill s-t edges
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        float ws = c.cellSWeight;
        float wt = c.cellTWeight;

        assert(ws >= 0.0f);
        assert(wt >= 0.0f);
        assert(!std::isnan(ws));
        assert(!std::isnan(wt));

        maxFlowGraph.addNode(ci, ws, wt);
    }

    ALICEVISION_LOG_INFO("Maxflow: add edges.");
    const float CONSTalphaVIS = 1.0f;
    const float CONSTalphaPHOTO = 5.0f;

    // fill u-v directed edges
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
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
    const std::size_t nbCells = _cellsAttr.size();
    std::vector<GC_cellInfo>().swap(_cellsAttr); // force clear

    long t_maxflow_compute = clock();
    // Find graph-cut solution
    ALICEVISION_LOG_INFO("Maxflow: compute.");
    const float totalFlow = maxFlowGraph.compute();
    mvsUtils::printfElapsedTime(t_maxflow_compute, "Maxflow computation ");
    ALICEVISION_LOG_INFO("totalFlow: " << totalFlow);

    ALICEVISION_LOG_INFO("Maxflow: update full/empty cells status.");
    _cellIsFull.resize(nbCells);
    // Update FULL/EMPTY status of all cells
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        _cellIsFull[ci] = maxFlowGraph.isTarget(ci);
    }

    mvsUtils::printfElapsedTime(t_maxflow, "Full maxflow step");

    ALICEVISION_LOG_INFO("Maxflow: done.");
}

void DelaunayGraphCut::reconstructExpetiments(const StaticVector<int>& cams, const std::string& folderName,
                                            bool update, Point3d* hexahInflated, const std::string& tmpCamsPtsFolderName,
                                            const Point3d& spaceSteps)
{
    ALICEVISION_LOG_INFO("DelaunayGraphCut::reconstructExpetiments");
    int maxint = 1000000.0f;

    long t1;

    bool fixesSigma = (update && spaceSteps.size() != 0.0);
    float sigma = (float)mp->userParams.get<double>("delaunaycut.sigma", 4.0f); // TODO FACA: 2 or 4?
    if(fixesSigma)
        sigma *= spaceSteps.size();

    ALICEVISION_LOG_INFO("fixesSigma: " << fixesSigma);
    ALICEVISION_LOG_INFO("sigma: " << sigma);

    // 0 for distFcn equals 1 all the time
    float distFcnHeight = (float)mp->userParams.get<double>("delaunaycut.distFcnHeight", 0.0f);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool labatutCFG09 = mp->userParams.get<bool>("global.LabatutCFG09", false);
    bool jancosekCVPR11 = mp->userParams.get<bool>("global.JancosekCVPR11", false);
    // jancosekIJCV: "Exploiting Visibility Information in Surface Reconstruction to Preserve Weakly Supported Surfaces", Michal Jancosek and Tomas Pajdla, 2014
    bool jancosekIJCV = mp->userParams.get<bool>("global.JancosekIJCV", true);

    if(jancosekCVPR11)
    {
        float delta = (float)mp->userParams.get<double>("delaunaycut.delta", 0.1f);

        ALICEVISION_LOG_INFO("Jancosek CVPR 2011 method ( delta*100 = " << static_cast<int>(delta * 100.0f) << "):");

        fillGraph(fixesSigma, sigma, true, false, false, true, distFcnHeight);

        if(update)
        {
            t1 = clock();
            updateGraphFromTmpPtsCamsHexah(cams, hexahInflated, tmpCamsPtsFolderName, false, distFcnHeight);
            mvsUtils::printfElapsedTime(t1);
        }
        addToInfiniteSw((float)maxint);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoInit.bin");

        if((delta > 0.0f) && (delta < 1.0f))
        {
            forceTedgesByGradientCVPR11(fixesSigma, sigma);
        }

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterForce.bin");

        reconstructGC(hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }

    if(jancosekIJCV) // true by default
    {
        float delta = (float)mp->userParams.get<double>("delaunaycut.delta", 0.1f);

        ALICEVISION_LOG_INFO("Jancosek IJCV method ( delta*100 = " << static_cast<int>(delta * 100.0f) << " ): ");

        // compute weights on edge between tetrahedra
        fillGraph(fixesSigma, sigma, true, false, false, true, distFcnHeight);

        if(update) // true by default
        {
            t1 = clock();
            updateGraphFromTmpPtsCamsHexah(cams, hexahInflated, tmpCamsPtsFolderName, false, distFcnHeight);
            mvsUtils::printfElapsedTime(t1);
        }
        addToInfiniteSw((float)maxint);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoInit.bin");

        if((delta > 0.0f) && (delta < 1.0f))
        {
            forceTedgesByGradientIJCV(fixesSigma, sigma);
        }

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterForce.bin");

        reconstructGC(hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }

    if(labatutCFG09)
    {
        ALICEVISION_LOG_INFO("Labatut CFG 2009 method:");
        fillGraph(fixesSigma, sigma, true, false, true, true, distFcnHeight);
        if(update)
        {
            t1 = clock();
            updateGraphFromTmpPtsCamsHexah(cams, hexahInflated, tmpCamsPtsFolderName, distFcnHeight != 0.0f);
            mvsUtils::printfElapsedTime(t1);
        }

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoInit.bin");

        reconstructGC(hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }
}

mesh::Mesh* DelaunayGraphCut::createMesh(bool filterHelperPointsTriangles)
{
    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut.");

    int nbSurfaceFacets = setIsOnSurface();

    ALICEVISION_LOG_INFO("# surface facets: " << nbSurfaceFacets);
    ALICEVISION_LOG_INFO("# vertixes: " << _verticesCoords.size());
    ALICEVISION_LOG_INFO("_cellIsFull.size(): " << _cellIsFull.size());

    mesh::Mesh* me = new mesh::Mesh();

    // TODO: copy only surface points and remap visibilities
    me->pts = new StaticVector<Point3d>();
    me->pts->reserve(_verticesCoords.size());

    for(const Point3d& p: _verticesCoords)
    {
        me->pts->push_back(p);
    }

    std::vector<bool> reliableVertices;
    if(filterHelperPointsTriangles)
    {
        // Some vertices have not been created by depth maps but
        // have been created for the tetrahedralization like
        // camera centers, helper points, etc.
        // These points have no visibility information (nrc == 0).
        // We want to remove these fake points, but we don't want to create holes.
        // So if the vertex is alone in the middle of valid points, we want to keep it.
        //
        // Algo: For each surface vertex without visibility,
        // we check the neighbor surface vertices. If there is another one
        // without visibility, we declare it unreliable.
        reliableVertices.resize(_verticesCoords.size());
        for(VertexIndex vi = 0; vi < _verticesCoords.size(); ++vi)
        {
            if(!_verticesAttr[vi].isOnSurface)
            {
                // this vertex is not on the surface, so no interest to spend time here
                reliableVertices[vi] = false;
            }
            else if(_verticesAttr[vi].nrc > 0)
            {
                // this is a valid point without ambiguity
                reliableVertices[vi] = true;
            }
            else
            {
                // this vertex has no visibility, check if it is connected to other weak vertices
                reliableVertices[vi] = true; // reliable by default
                GEO::vector<GEO::index_t> neighbors;
                _tetrahedralization->get_neighbors(vi, neighbors);
                for(GEO::index_t nvi: neighbors)
                {
                    if(_verticesAttr[nvi].isOnSurface && (_verticesAttr[nvi].nrc == 0))
                    {
                        // this vertex has no visibility and is connected to another
                        // surface vertex without visibility, so we declare it unreliable.
                        reliableVertices[vi] = false;
                        break;
                    }
                }
            }
        }
    }

    me->tris = new StaticVector<mesh::Mesh::triangle>();
    me->tris->reserve(nbSurfaceFacets);

    // loop over all facets
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellIsFull[f1.cellIndex]; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellIsFull[f2.cellIndex]; // get if it is occupied

            if(uo == vo)
                continue;

            VertexIndex vertices[3];
            vertices[0] = getVertexIndex(f1, 0);
            vertices[1] = getVertexIndex(f1, 1);
            vertices[2] = getVertexIndex(f1, 2);

            if(filterHelperPointsTriangles)
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

            Point3d D1 = _verticesCoords[getOppositeVertexIndex(f1)];
            Point3d D2 = _verticesCoords[getOppositeVertexIndex(f2)];

            Point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

            float dd1 = orientedPointPlaneDistance(D1, points[0], N);
            float dd2 = orientedPointPlaneDistance(D2, points[0], N);

            bool clockwise = false;
            if(dd1 == 0.0f)
            {
                if(dd2 == 0.0f)
                {
                    ALICEVISION_LOG_WARNING("createMesh: bad triangle orientation.");
                }
                if(dd2 > 0.0f)
                {
                    clockwise = true;
                }
            }
            else
            {
                if(dd1 < 0.0f)
                {
                    clockwise = true;
                }
            }

            if(clockwise)
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertices[0];
                t.v[1] = vertices[1];
                t.v[2] = vertices[2];
                me->tris->push_back(t);
            }
            else
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertices[0];
                t.v[1] = vertices[2];
                t.v[2] = vertices[1];
                me->tris->push_back(t);
            }
        }
    }

    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut done.");
    return me;
}

void DelaunayGraphCut::segmentFullOrFree(bool full, StaticVector<int>** out_fullSegsColor, int& out_nsegments)
{
    ALICEVISION_LOG_DEBUG("segmentFullOrFree: segmenting connected space.");

    StaticVector<int>* colors = new StaticVector<int>();
    colors->reserve(_cellIsFull.size());
    colors->resize_with(_cellIsFull.size(), -1);

    StaticVector<CellIndex>* buff = new StaticVector<CellIndex>();
    buff->reserve(_cellIsFull.size());
    int col = 0;

    // segment connected free space
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if((!isInfiniteCell(ci)) && ((*colors)[ci] == -1) && (_cellIsFull[ci] == full))
        {
            // backtrack all connected interior cells
            buff->resize(0);
            buff->push_back(ci);

            while(buff->size() > 0)
            {
                CellIndex tmp_ci = buff->pop();

                (*colors)[tmp_ci] = col;

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    if((!isInfiniteCell(nci)) && ((*colors)[nci] == -1) &&
                       (_cellIsFull[nci] == full))
                    {
                        buff->push_back(nci);
                    }
                }
            }
            ++col;
        }
    }

    delete buff;

    *out_fullSegsColor = colors;
    out_nsegments = col;
}

int DelaunayGraphCut::removeBubbles()
{
    int nbEmptySegments = 0;
    StaticVector<int>* emptySegColors = nullptr;
    segmentFullOrFree(false, &emptySegColors, nbEmptySegments);

    ALICEVISION_LOG_DEBUG("removing bubbles.");

    StaticVectorBool* colorsToFill = new StaticVectorBool();
    colorsToFill->reserve(nbEmptySegments);
    // all free space segments which contains camera has to remain free all others full
    colorsToFill->resize_with(nbEmptySegments, true);

    // all free space segments which contains camera has to remain free
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if(isInfiniteCell(ci) || (*emptySegColors)[ci] < 0)
            continue;

        const GC_vertexInfo& a = _verticesAttr[_tetrahedralization->cell_vertex(ci, 0)];
        const GC_vertexInfo& b = _verticesAttr[_tetrahedralization->cell_vertex(ci, 1)];
        const GC_vertexInfo& c = _verticesAttr[_tetrahedralization->cell_vertex(ci, 2)];
        const GC_vertexInfo& d = _verticesAttr[_tetrahedralization->cell_vertex(ci, 3)];
        if( a.isVirtual() || b.isVirtual() || c.isVirtual() || d.isVirtual())
        {
            // TODO FACA: check helper points are not connected to cameras?
            (*colorsToFill)[(*emptySegColors)[ci]] = false;
        }
    }

    int nbubbles = 0;
    for(int i = 0; i < nbEmptySegments; ++i)
    {
        if((*colorsToFill)[i])
        {
            ++nbubbles;
        }
    }

    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if((!isInfiniteCell(ci)) && ((*emptySegColors)[ci] >= 0) && ((*colorsToFill)[(*emptySegColors)[ci]]))
        {
            _cellIsFull[ci] = true;
        }
    }

    delete colorsToFill;
    delete emptySegColors;

    ALICEVISION_LOG_DEBUG("nbubbles: " << nbubbles << ", all empty segments: " << nbEmptySegments);

    setIsOnSurface();

    return nbubbles;
}

int DelaunayGraphCut::removeDust(int minSegSize)
{
    ALICEVISION_LOG_DEBUG("removing dust.");

    int nbFullSegments = 0;
    StaticVector<int>* fullSegsColor = nullptr;
    segmentFullOrFree(true, &fullSegsColor, nbFullSegments);

    StaticVector<int>* colorsSize = new StaticVector<int>();
    colorsSize->reserve(nbFullSegments);
    colorsSize->resize_with(nbFullSegments, 0);

    // all free space segments which contains camera has to remain free
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if((*fullSegsColor)[ci] >= 0) // if we have a valid color: non empty and non infinit cell
        {
            (*colorsSize)[(*fullSegsColor)[ci]] += 1; // count the number of cells in the segment
        }
    }

    int ndust = 0;
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        // if number of cells in the segment is too small, we change the status to "empty"
        if(((*fullSegsColor)[ci] >= 0) && ((*colorsSize)[(*fullSegsColor)[ci]] < minSegSize))
        {
            _cellIsFull[ci] = false;
            ++ndust;
        }
    }

    delete colorsSize;
    delete fullSegsColor;

    ALICEVISION_LOG_DEBUG("Removed dust cells: " << ndust << ", Number of segments: " << nbFullSegments);

    setIsOnSurface();

    return ndust;
}

void DelaunayGraphCut::leaveLargestFullSegmentOnly()
{
    ALICEVISION_LOG_DEBUG("Largest full segment only.");

    int nsegments;
    StaticVector<int>* colors = nullptr;
    segmentFullOrFree(true, &colors, nsegments);

    StaticVector<int>* colorsSize = new StaticVector<int>();
    colorsSize->reserve(nsegments);
    colorsSize->resize_with(nsegments, 0);

    // all free space segments which contains camera has to remain free
    int largestColor = -1;
    int maxn = 0;
    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        int color = (*colors)[ci];
        if(color >= 0)
        {
            (*colorsSize)[color] += 1;
            int n = (*colorsSize)[color];
            if(n > maxn)
            {
                maxn = n;
                largestColor = color;
            }
        }
    }

    for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if((*colors)[ci] != largestColor)
        {
            _cellIsFull[ci] = false;
        }
    }

    delete colorsSize;
    delete colors;

    setIsOnSurface();

    ALICEVISION_LOG_DEBUG("Largest full segment only done.");
}

} // namespace fuseCut
} // namespace aliceVision
