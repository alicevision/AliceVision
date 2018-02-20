// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DelaunayGraphCut.hpp"
#include <aliceVision/delaunayCut/MaxFlow_CSR.hpp>
// #include <aliceVision/delaunayCut/MaxFlow_AdjList.hpp>
#include <aliceVision/structures/geometry.hpp>
#include <aliceVision/structures/jetColorMap.hpp>
#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Universe.hpp>
#include <aliceVision/common/fileIO.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

// OpenMP >= 3.1 for advanced atomic clauses (https://software.intel.com/en-us/node/608160)
// OpenMP preprocessor version: https://github.com/jeffhammond/HPCInfo/wiki/Preprocessor-Macros
#if defined _OPENMP && _OPENMP >= 201107 
#define OMP_ATOMIC_UPDATE _Pragma("omp atomic update")
#define OMP_ATOMIC_WRITE  _Pragma("omp atomic write")
#else
#define OMP_ATOMIC_UPDATE _Pragma("omp atomic")
#define OMP_ATOMIC_WRITE  _Pragma("omp atomic")
#endif

namespace aliceVision {
namespace delaunayCut {

namespace bfs = boost::filesystem;

DelaunayGraphCut::DelaunayGraphCut(MultiViewParams* _mp, PreMatchCams* _pc)
{
    mp = _mp;
    pc = _pc;

    _camsVertexes.resize(mp->ncams, -1);

    saveTemporaryBinFiles = mp->mip->_ini.get<bool>("LargeScale.saveTemporaryBinFiles", false);

    GEO::initialize();
    _tetrahedralization = GEO::Delaunay::create(3, "BDEL");
    // _tetrahedralization->set_keeps_infinite(true);
    _tetrahedralization->set_stores_neighbors(true);
    // _tetrahedralization->set_stores_cicl(true);
}

DelaunayGraphCut::~DelaunayGraphCut()
{
}

void DelaunayGraphCut::saveDhInfo(std::string fileNameInfo)
{
    FILE* f = fopen(fileNameInfo.c_str(), "wb");

    int npts = getNbVertices();
    fwrite(&npts, sizeof(int), 1, f);
    for(const GC_vertexInfo& v: _verticesAttr)
    {
        v.fwriteinfo(f);
    }

    if(mp->verbose)
        printf(".");

    int ncells = _cellsAttr.size();
    fwrite(&ncells, sizeof(int), 1, f);
    for(const GC_cellInfo& c: _cellsAttr)
    {
        c.fwriteinfo(f);
    }
    fclose(f);

    if(mp->verbose)
        printf(".");
}

void DelaunayGraphCut::saveDh(std::string fileNameDh, std::string fileNameInfo)
{
    if(mp->verbose)
        printf("Saving triangulation");

    saveDhInfo(fileNameInfo);

    long t1 = clock();
    // std::ofstream oFileT(fileNameDh.c_str());
    // oFileT << *_tetrahedralization; // TODO GEOGRAM

    printfElapsedTime(t1);
}

void DelaunayGraphCut::initVertices()
{
    if(mp->verbose)
        printf("initVertices ...\n");

    // Re-assign ids to the vertices to go one after another
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        GC_vertexInfo& v = _verticesAttr[vi];
        // fit->info().point = convertPointToPoint3d(fit->point());
        v.isOnSurface = false;
        // v.id = nVertices;
        v.pixSize = mp->getCamsMinPixelSize(_verticesCoords[vi], v.cams);
    }

    if(mp->verbose)
        printf("initVertices done\n");
}

void DelaunayGraphCut::computeDelaunay()
{
    if(mp->verbose)
        printf("computeDelaunay GEOGRAM ...\n");

    assert(_verticesCoords.size() == _verticesAttr.size());

    long tall = clock();
    _tetrahedralization->set_vertices(_verticesCoords.size(), _verticesCoords.front().m);
    printfElapsedTime(tall, "GEOGRAM Delaunay tetrahedralization ");

    initCells();

    updateVertexToCellsCache();

    if(mp->verbose)
        printf("computeDelaunay done\n");
}

void DelaunayGraphCut::initCells()
{
    if(mp->verbose)
        printf("initCells ...\n");

    _cellsAttr.resize(_tetrahedralization->nb_cells()); // or nb_finite_cells() if keeps_infinite()

    std::cout << _cellsAttr.size() << " cells created by tetrahedralization." << std::endl;
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

    if(mp->verbose)
        printf("initCells done\n");
}

void DelaunayGraphCut::displayStatistics()
{
    // Display some statistics

    StaticVector<int>* ptsCamsHist = getPtsCamsHist();
    std::cout << "Histogram of number of cams per point:\n";
    for(int i = 0; i < ptsCamsHist->size(); ++i)
        std::cout << "    " << i << ": " << num2str((*ptsCamsHist)[i]) << "\n";
    std::cout << "\n";
    delete ptsCamsHist;

    StaticVector<int>* ptsNrcsHist = getPtsNrcHist();
    std::cout << "Histogram of alpha_vis per point:\n";
    for(int i = 0; i < ptsNrcsHist->size(); ++i)
        std::cout << "    " << i << ": " << num2str((*ptsNrcsHist)[i]) << "\n";
    std::cout << "\n";
    delete ptsNrcsHist;
}

StaticVector<StaticVector<int>*>* DelaunayGraphCut::createPtsCams()
{
    long t = std::clock();
    std::cout << "Extract visibilities." << std::endl;
    int npts = getNbVertices();
    StaticVector<StaticVector<int>*>* out = new StaticVector<StaticVector<int>*>(npts);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        StaticVector<int>* cams = new StaticVector<int>(v.getNbCameras());
        for(int c = 0; c < v.getNbCameras(); c++)
        {
            cams->push_back(v.cams[c]);
        }
        out->push_back(cams);
    } // for i

    std::cout << "Extract visibilities done." << std::endl;

    printfElapsedTime(t, "Extract visibilities ");
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
    if(mp->verbose)
        printf("maxnCams %i\n", maxnCams);

    StaticVector<int>* ncamsHist = new StaticVector<int>(maxnCams);
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
    if(mp->verbose)
        printf("maxnnrcs %i\n", maxnnrcs);
    maxnnrcs = std::min(1000, maxnnrcs);
    if(mp->verbose)
        printf("maxnnrcs %i\n", maxnnrcs);

    StaticVector<int>* nnrcsHist = new StaticVector<int>(maxnnrcs);
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
    cams.resize_with(mp->mip->getNbCameras(), 0);

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

    printfElapsedTime(timer, "getIsUsedPerCamera ");
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

void DelaunayGraphCut::addPointsFromCameraCenters(StaticVector<int>* cams, float minDist)
{
    for(int camid = 0; camid < cams->size(); camid++)
    {
        int rc = (*cams)[camid];
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

void DelaunayGraphCut::addPointsToPreventSingularities(Point3d voxel[8], float minDist)
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

void DelaunayGraphCut::addHelperPoints(int nGridHelperVolumePointsDim, Point3d voxel[8], float minDist)
{
    if(nGridHelperVolumePointsDim <= 0)
        return;

    if(mp->verbose)
        printf("adding helper points ...");

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

    if(mp->verbose)
        printf(" done\n");
}


void DelaunayGraphCut::createTetrahedralizationFromDepthMapsCamsVoxel(StaticVector<int>* cams,
                                                               StaticVector<int>* voxelsIds, Point3d voxel[8],
                                                               largeScale::VoxelsGrid* ls)
{
    ///////////////////////////////////////////////////////////////////////////////////////
    printf("Creating delaunay tetrahedralization from depth maps voxel\n");
    long tall = clock();

    bool doFilterOctreeTracks = mp->mip->_ini.get<bool>("LargeScale.doFilterOctreeTracks", true);
    int minNumOfConsistentCams = mp->mip->_ini.get<int>("filter.minNumOfConsistentCams", 2);

    ///////////////////////////////////////////////////////////////////////////////////////
    // build tetrahedralization

    float minDist = (voxel[0] - voxel[1]).size() / 1000.0f;

    // add points for cam centers
    addPointsFromCameraCenters(cams, minDist);

    // add 6 points to prevent singularities
    addPointsToPreventSingularities(voxel, minDist);

    Point3d cgpt;
    int ncgpt = 0;

    // add points from voxel
    for(int i = 0; i < voxelsIds->size(); i++)
    {
        printf("%i of %i\n", i, voxelsIds->size());

        std::string folderName = ls->getVoxelFolderName((*voxelsIds)[i]);

        std::string fileNameTracksCams, fileNameTracksPts, fileNameTracksPtsCams;
        fileNameTracksCams = folderName + "tracksGridCams.bin";
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";

        if(FileExists(fileNameTracksPts))
        {
            StaticVector<Point3d>* tracksPoints = loadArrayFromFile<Point3d>(fileNameTracksPts);
            StaticVector<StaticVector<Pixel>*>* tracksPointsCams =
                loadArrayOfArraysFromFile<Pixel>(fileNameTracksPtsCams);

            long t1 = initEstimate();
            for(int j = 0; j < tracksPoints->size(); j++)
            {
                Point3d tp = (*tracksPoints)[j];
                StaticVector<Pixel>* cams = (*tracksPointsCams)[j];
                if((isPointInHexahedron(tp, voxel)) && (cams != nullptr) &&
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

                printfEstimate(j, tracksPoints->size(), t1);
            } // for j
            finishEstimate();

            // delete randIds;

            delete tracksPoints;
            deleteArrayOfArrays<Pixel>(&tracksPointsCams);
        } // if fileexists
    }

    if(mp->verbose)
        printf("voxels tracks triangulated\n");

    /* initialize random seed: */
    srand(time(nullptr));

    int nGridHelperVolumePointsDim = mp->mip->_ini.get<int>("LargeScale.nGridHelperVolumePointsDim", 10);
    // add volume points to prevent singularities
    addHelperPoints(nGridHelperVolumePointsDim, voxel, minDist);

    initVertices();

    computeDelaunay();

    printfElapsedTime(tall, "Create Delaunay tetrahedralization from depth map voxels ");

    displayStatistics();
}

void DelaunayGraphCut::computeVerticesSegSize(bool allPoints, float alpha) // allPoints=true, alpha=0
{
    if(mp->verbose)
        printf("creating universe\n");
    int scalePS = mp->mip->_ini.get<int>("global.scalePS", 1);
    int step = mp->mip->_ini.get<int>("global.step", 1);
    float pointToJoinPixSizeDist = (float)mp->mip->_ini.get<double>("delaunaycut.pointToJoinPixSizeDist", 2.0) *
                                   (float)scalePS * (float)step * 2.0f;

    std::vector<Pixel> edges;
    edges.reserve(_verticesAttr.size());

    if(alpha < 1.0f)
    {
        alpha = 2.0f * std::max(2.0f, pointToJoinPixSizeDist);
    }

    long t1 = initEstimate();
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
        printfEstimate(vi, _verticesAttr.size(), t1);
    }
    finishEstimate();

    Universe* u = new Universe(_verticesAttr.size());

    t1 = initEstimate();
    int s = (int)edges.size(); // Fuse all edges collected to be merged
    for(int i = 0; i < s; i++)
    {
        int a = u->find(edges[i].x);
        int b = u->find(edges[i].y);
        if(a != b) // TODO FACA: Are they not different in all cases?
        {
            u->join(a, b);
        }
        printfEstimate(i, s, t1);
    }
    finishEstimate();

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
    if(mp->verbose)
        printf("creating universe done.\n");
}

void DelaunayGraphCut::removeSmallSegs(int minSegSize)
{
    if(mp->verbose)
        std::cout << "removeSmallSegs: " << minSegSize << std::endl;
    StaticVector<int>* toRemove = new StaticVector<int>(getNbVertices());

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
    printf("Computing s-t graph weights\n");
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
    StaticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

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

    if(mp->verbose)
    {
        std::cout << "avStepsFront " << avStepsFront << " \n";
        std::cout << "avStepsFront = " << num2str(avStepsFront) << " // " << num2str(aAvStepsFront) << " \n";
        std::cout << "avStepsBehind = " << num2str(avStepsBehind) << " // " << num2str(nAvStepsBehind) << " \n";
        std::cout << "avCams = " << num2str(avCams) << " // " << num2str(nAvCams) << " \n";
    }
    printfElapsedTime(t1, "s-t graph weights computed : ");
}

void DelaunayGraphCut::fillGraphPartPtRc(int& out_nstepsFront, int& out_nstepsBehind, int vertexIndex, int cam,
                                       float weight, bool fixesSigma, float nPixelSizeBehind, bool allPoints,
                                       bool behind, bool fillOut, float distFcnHeight)  // fixesSigma=true nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 fillOut=1 distFcnHeight=0
{
    out_nstepsFront = 0;
    out_nstepsBehind = 0;

    int maxint = 1000000.0f; // std::numeric_limits<int>::std::max()

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
    printf("Forcing t-edges\n");
    long t2 = clock();

    float delta = (float)mp->mip->_ini.get<double>("delaunaycut.delta", 0.1f);
    printf("delta %f \n", delta);

    float beta = (float)mp->mip->_ini.get<double>("delaunaycut.beta", 1000.0f);
    printf("beta %f \n", beta);

    for(GC_cellInfo& c: _cellsAttr)
    {
        c.on = 0.0f;
    }

    // choose random order to prevent waiting
    StaticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

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

    printfElapsedTime(t2, "t-edges forced : ");
}

void DelaunayGraphCut::forceTedgesByGradientIJCV(bool fixesSigma, float nPixelSizeBehind)
{
    printf("Forcing t-edges\n");
    long t2 = clock();

    float delta = (float)mp->mip->_ini.get<double>("delaunaycut.delta", 0.1f);
    if(mp->verbose)
        printf("delta %f \n", delta);

    float minJumpPartRange = (float)mp->mip->_ini.get<double>("delaunaycut.minJumpPartRange", 10000.0f);
    if(mp->verbose)
        printf("minJumpPartRange %f \n", minJumpPartRange);

    float maxSilentPartRange = (float)mp->mip->_ini.get<double>("delaunaycut.maxSilentPartRange", 100.0f);
    if(mp->verbose)
        printf("maxSilentPartRange %f \n", maxSilentPartRange);

    float nsigmaJumpPart = (float)mp->mip->_ini.get<double>("delaunaycut.nsigmaJumpPart", 2.0f);
    if(mp->verbose)
        printf("nsigmaJumpPart %f \n", nsigmaJumpPart);

    float nsigmaFrontSilentPart = (float)mp->mip->_ini.get<double>("delaunaycut.nsigmaFrontSilentPart", 2.0f);
    if(mp->verbose)
        printf("nsigmaFrontSilentPart %f \n", nsigmaFrontSilentPart);

    float nsigmaBackSilentPart = (float)mp->mip->_ini.get<double>("delaunaycut.nsigmaBackSilentPart", 2.0f);
    if(mp->verbose)
        printf("nsigmaBackSilentPart %f \n", nsigmaBackSilentPart);


    for(GC_cellInfo& c: _cellsAttr)
    {
        c.on = 0.0f;
        // WARNING out is not the same as the sum because the sum are counted edges behind as well
        // c.out = c.gEdgeVisWeight[0] + c.gEdgeVisWeight[1] + c.gEdgeVisWeight[2] + c.gEdgeVisWeight[3];
    }

    // choose random order to prevent waiting
    StaticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

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

    if(mp->verbose)
    {
        std::string sstat;
        sstat = "avStepsFront = " + num2str(avStepsFront) + " // " + num2str(aAvStepsFront) + " \n";
        printf("%s", sstat.c_str());
        sstat = "avStepsBehind = " + num2str(avStepsBehind) + " // " + num2str(nAvStepsBehind) + " \n";
        printf("%s", sstat.c_str());
    }
    printfElapsedTime(t2, "t-edges forced : ");
}

void DelaunayGraphCut::updateGraphFromTmpPtsCamsHexah(StaticVector<int>* incams, Point3d hexah[8],
                                                    std::string tmpCamsPtsFolderName, bool labatutWeights,
                                                    float distFcnHeight)
{
    printf("Updating : LSC\n");

#pragma omp parallel for
    for(int c = 0; c < incams->size(); c++)
    {
        int rc = (*incams)[c];
        std::string camPtsFileName;
        camPtsFileName = tmpCamsPtsFolderName + "camPtsGrid_" + num2strFourDecimal(rc) + ".bin";
        if(FileExists(camPtsFileName))
        {
            updateGraphFromTmpPtsCamsHexahRC(rc, hexah, tmpCamsPtsFolderName, labatutWeights, distFcnHeight);
        }
    } // for c
}

void DelaunayGraphCut::updateGraphFromTmpPtsCamsHexahRC(int rc, Point3d hexah[8], std::string tmpCamsPtsFolderName,
                                                      bool labatutWeights, float  /*distFcnHeight*/)
{

    // fill edges
    int nin = 0;
    int nout = 0;
    int cnull = 0;
    int nwup = 0;

    std::string camPtsFileName;
    camPtsFileName = tmpCamsPtsFolderName + "camPtsGrid_" + num2strFourDecimal(rc) + ".bin";

    bool doFilterOctreeTracks = mp->mip->_ini.get<bool>("LargeScale.doFilterOctreeTracks", true);
    int minNumOfConsistentCams = mp->mip->_ini.get<int>("filter.minNumOfConsistentCams", 2);

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

        StaticVector<Point3d>* lshi = lineSegmentHexahedronIntersection(pt, mp->CArr[rc], hexah);
        if((!isPointInHexahedron(pt, hexah)) && (lshi->size() >= 1) && (feof(f) == 0) &&
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

    // if (mp->verbose) printf("in %i, cnull %i, nwup %i, out
    // %i\n",nin,cnull,nwup,nout);
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
    std::cout << "setIsOnSurface nbSurfaceFacets: " << nbSurfaceFacets << std::endl;
    return nbSurfaceFacets;
}

void DelaunayGraphCut::graphCutPostProcessing()
{
    long timer = std::clock();
    std::cout << "Graph cut post-processing." << std::endl;
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
    std::cout << "Graph cut post-processing done." << std::endl;

    printfElapsedTime(timer, "Graph cut post-processing ");
}

void DelaunayGraphCut::freeUnwantedFullCells(std::string  /*folderName*/, Point3d* hexah)
{
    if(mp->verbose)
        printf("freeUnwantedFullCells\n");

    int minSegmentSize = (int)mp->mip->_ini.get<int>("hallucinationsFiltering.minSegmentSize", 10);
    bool doRemoveBubbles = (bool)mp->mip->_ini.get<bool>("hallucinationsFiltering.doRemoveBubbles", true);
    bool doRemoveDust = (bool)mp->mip->_ini.get<bool>("hallucinationsFiltering.doRemoveDust", true);
    bool doLeaveLargestFullSegmentOnly = (bool)mp->mip->_ini.get<bool>("hallucinationsFiltering.doLeaveLargestFullSegmentOnly", false);

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
        inflateHexahedron(hexah, hexahinf, 1.001);
        for(CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
        {
            if(isInfiniteCell(ci) || !_cellIsFull[ci])
                continue;

            const Point3d& pa = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
            const Point3d& pb = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
            const Point3d& pc = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
            const Point3d& pd = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];

            if((!isPointInHexahedron(pa, hexahinf)) ||
               (!isPointInHexahedron(pb, hexahinf)) ||
               (!isPointInHexahedron(pc, hexahinf)) ||
               (!isPointInHexahedron(pd, hexahinf)))
            {
                _cellIsFull[ci] = false;
                ++nremoved;
            }
        }
        if(mp->verbose)
            printf("%i removed cells outside hexahedron\n", nremoved);
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
    if(mp->verbose)
        printf("filling small holes\n");

    const std::size_t nbCells = _cellIsFull.size();
    StaticVector<int>* colorPerCell = new StaticVector<int>(nbCells);
    colorPerCell->resize_with(nbCells, -1);

    StaticVector<int>* nbCellsPerColor = new StaticVector<int>(100);
    nbCellsPerColor->resize_with(1, 0);
    int lastColorId = 0;

    StaticVector<CellIndex>* buff = new StaticVector<CellIndex>(nbCells);

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

    if(mp->verbose)
        std::cout << "Full number of cells: " << nbCells << ", Number of labels: " << nbCellsPerColor->size() << ", Number of cells changed: " << nfilled << std::endl;

    delete nbCellsPerColor;
    delete colorPerCell;
}

void DelaunayGraphCut::reconstructVoxel(Point3d hexah[8], StaticVector<int>* voxelsIds, std::string folderName,
                                      std::string tmpCamsPtsFolderName, bool removeSmallSegments,
                                      StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, largeScale::VoxelsGrid* ls,
                                      Point3d spaceSteps)
{
    StaticVector<int>* cams = pc->findCamsWhichIntersectsHexahedron(hexah);

    if(cams->size() < 1)
        throw std::logic_error("No camera to make the reconstruction");

    int camerasPerOneOmni = 1;
    std::string fileNameDh = folderName + "delaunayTriangulation.bin";
    std::string fileNameInfo = folderName + "delaunayTriangulationInfo.bin";
    std::string fileNameStGraph = folderName + "stGraph.bin";
    std::string fileNameStSolution = folderName + "stGraphSolution.bin";
    std::string fileNameTxt = folderName + "delaunayTrianglesMaxflow.txt";
    std::string fileNameTxtCam = folderName + "delaunayTrianglesCamerasForColoringMaxflow.txt";

    // Load tracks and create tetrahedralization (into T variable)
    createTetrahedralizationFromDepthMapsCamsVoxel(cams, voxelsIds, hexah, ls);

    //float nPixelSizeBehind = (float)mp->mip->_ini.get<double>("delaunaycut.filterPtsWithHigherPixSizeInDist", 5.0f)*spaceSteps.size();
    // initTriangulationDefaults(folderName + "delaunayVerticesFiltered.wrl");

    computeVerticesSegSize(true, 0.0f);
    if(removeSmallSegments) // false
    {
        removeSmallSegs(2500); // TODO FACA: to decide
    }

    bool updateLSC = mp->mip->_ini.get<bool>("LargeScale.updateLSC", true);

    reconstructExpetiments(cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt, fileNameTxtCam,
                           camerasPerOneOmni, updateLSC, hexah, tmpCamsPtsFolderName, hexahsToExcludeFromResultingMesh,
                           spaceSteps);

    bool saveOrNot = mp->mip->_ini.get<bool>("LargeScale.saveDelaunayTriangulation", false);
    if(saveOrNot)
    {
        saveDh(fileNameDh, fileNameInfo);
    }

    // reconstructExpetiments(cams, folderName, fileNameStGraph,
    // fileNameStSolution, fileNameTxt,
    // fileNameTxtCam, camerasPerOneOmni, true, hexahInflated,
    // tmpCamsPtsFolderName);

    // smooth(mp,fileNameTxt,"delaunayTrianglesSmoothTextured.wrl","delaunayTrianglesSmoothTextured.ply",folderName,1,true);
    // filterLargeTrianglesMeshDist(mp, pc, folderName,
    // "meshTrisAreaColored.wrl",
    // "meshAreaConsistentTextured.wrl", "meshAreaConsistent.ply",
    // "meshAreaConsistent.wrl");

    delete cams;
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

void DelaunayGraphCut::reconstructGC(float alphaQual, std::string baseName, StaticVector<int>* cams,
                                   std::string folderName, std::string fileNameStGraph, std::string fileNameStSolution,
                                   std::string fileNameTxt, std::string fileNameTxtCam, int camerasPerOneOmni,
                                   bool doRemoveBubbles, StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d* hexah) // alphaQual=5.0f
{
    std::cout << "reconstructGC" << std::endl;

    maxflow();

    std::cout << "Maxflow: convert result to surface" << std::endl;
    // Convert cells FULL/EMPTY into surface
    setIsOnSurface();

    std::string resultFolderName = folderName + baseName + "/";
    bfs::create_directory(resultFolderName);

    freeUnwantedFullCells(resultFolderName, hexah);

    std::cout << "reconstructGC end" << std::endl;
}

void DelaunayGraphCut::maxflow()
{
    long t_maxflow = clock();

    std::cout << "Maxflow: start allocation" << std::endl;
    MaxFlow_CSR maxFlowGraph(_cellsAttr.size());

    std::cout << "Maxflow: add nodes" << std::endl;
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

    std::cout << "Maxflow: add edges" << std::endl;
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

    std::cout << "Maxflow: clear cells info" << std::endl;
    const std::size_t nbCells = _cellsAttr.size();
    std::vector<GC_cellInfo>().swap(_cellsAttr); // force clear

    long t_maxflow_compute = clock();
    // Find graph-cut solution
    std::cout << "Maxflow: compute" << std::endl;
    const float totalFlow = maxFlowGraph.compute();
    printfElapsedTime(t_maxflow_compute, "Maxflow computation ");
    std::cout << "totalFlow: " << totalFlow << std::endl;

    std::cout << "Maxflow: update full/empty cells status" << std::endl;
    _cellIsFull.resize(nbCells);
    // Update FULL/EMPTY status of all cells
    for(CellIndex ci = 0; ci < nbCells; ++ci)
    {
        _cellIsFull[ci] = maxFlowGraph.isTarget(ci);
    }

    printfElapsedTime(t_maxflow, "Full maxflow step");

    std::cout << "Maxflow: end" << std::endl;
}

void DelaunayGraphCut::reconstructExpetiments(StaticVector<int>* cams, std::string folderName,
                                            std::string fileNameStGraph, std::string fileNameStSolution,
                                            std::string fileNameTxt, std::string fileNameTxtCam, int camerasPerOneOmni,
                                            bool update, Point3d* hexahInflated, std::string tmpCamsPtsFolderName,
                                            StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d spaceSteps)
{
    int maxint = 1000000.0f;

    long t1;

    bool fixesSigma = update;
    float sigma = (float)mp->mip->_ini.get<double>("delaunaycut.sigma", 2.0f) * spaceSteps.size();

    // 0 for distFcn equals 1 all the time
    float distFcnHeight = (float)mp->mip->_ini.get<double>("delaunaycut.distFcnHeight", 0.0f);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool labatutCFG09 = mp->mip->_ini.get<bool>("global.LabatutCFG09", false);
    bool jancosekCVPR11 = mp->mip->_ini.get<bool>("global.JancosekCVPR11", false);
    // jancosekIJCV: "Exploiting Visibility Information in Surface Reconstruction to Preserve Weakly Supported Surfaces", Michal Jancosek and Tomas Pajdla, 2014
    bool jancosekIJCV = mp->mip->_ini.get<bool>("global.JancosekIJCV", true);

    if(jancosekCVPR11)
    {
        float delta = (float)mp->mip->_ini.get<double>("delaunaycut.delta", 0.1f);

        printf("Jancosek CVPR 2011 method ( delta*100 = %d ) : \n", (int)(delta * 100.0f));

        fillGraph(fixesSigma, sigma, true, false, false, true, distFcnHeight);

        if(update)
        {
            t1 = clock();
            updateGraphFromTmpPtsCamsHexah(cams, hexahInflated, tmpCamsPtsFolderName, false, distFcnHeight);
            printfElapsedTime(t1);
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

        reconstructGC(5.0f, "", cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt,
                      fileNameTxtCam, camerasPerOneOmni, true, hexahsToExcludeFromResultingMesh, hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }

    if(jancosekIJCV) // true by default
    {
        float delta = (float)mp->mip->_ini.get<double>("delaunaycut.delta", 0.1f);

        if(mp->verbose)
            printf("Jancosek IJCV method ( delta*100 = %d ) : \n", (int)(delta * 100.0f));

        // compute weights on edge between tetrahedra
        fillGraph(fixesSigma, sigma, true, false, false, true, distFcnHeight);

        if(update) // true by default
        {
            t1 = clock();
            updateGraphFromTmpPtsCamsHexah(cams, hexahInflated, tmpCamsPtsFolderName, false, distFcnHeight);
            printfElapsedTime(t1);
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

        reconstructGC(5.0f, "", cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt,
                      fileNameTxtCam, camerasPerOneOmni, true, hexahsToExcludeFromResultingMesh, hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }

    if(labatutCFG09)
    {
        printf("Labatut CFG 2009 method :\n");
        fillGraph(fixesSigma, sigma, true, false, true, true, distFcnHeight);
        if(update)
        {
            t1 = clock();
            updateGraphFromTmpPtsCamsHexah(cams, hexahInflated, tmpCamsPtsFolderName, distFcnHeight != 0.0f);
            printfElapsedTime(t1);
        }

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoInit.bin");

        reconstructGC(5.0f, "", cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt,
                      fileNameTxtCam, camerasPerOneOmni, true, hexahsToExcludeFromResultingMesh, hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }
}

mesh::Mesh* DelaunayGraphCut::createMesh(bool filterHelperPointsTriangles)
{
    std::cout << "Extract mesh from GC" << std::endl;

    int nbSurfaceFacets = setIsOnSurface();

    std::cout << "Nb surface facets: " << nbSurfaceFacets << std::endl;
    std::cout << "Nb vertixes: " << _verticesCoords.size() << std::endl;
    std::cout << "_cellIsFull.size(): " << _cellIsFull.size() << std::endl;

    mesh::Mesh* me = new mesh::Mesh();

    // TODO: copy only surface points and remap visibilities
    me->pts = new StaticVector<Point3d>(_verticesCoords.size());
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

    me->tris = new StaticVector<mesh::Mesh::triangle>(nbSurfaceFacets);
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
                    printf("WARNING bad triangle orientation.\n");
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
                t.i[0] = vertices[0];
                t.i[1] = vertices[1];
                t.i[2] = vertices[2];
                me->tris->push_back(t);
            }
            else
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.i[0] = vertices[0];
                t.i[1] = vertices[2];
                t.i[2] = vertices[1];
                me->tris->push_back(t);
            }
        }
    }

    std::cout << "Extract mesh from GC done." << std::endl;
    return me;
}

void DelaunayGraphCut::segmentFullOrFree(bool full, StaticVector<int>** out_fullSegsColor, int& out_nsegments)
{
    if(mp->verbose)
        printf("segmenting connected space\n");

    StaticVector<int>* colors = new StaticVector<int>(_cellIsFull.size());
    colors->resize_with(_cellIsFull.size(), -1);

    StaticVector<CellIndex>* buff = new StaticVector<CellIndex>(_cellIsFull.size());

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

    if(mp->verbose)
        printf("removing bubbles\n");

    StaticVectorBool* colorsToFill = new StaticVectorBool(nbEmptySegments);
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

    if(mp->verbose)
        printf("nbubbles %i, all segs %i\n", nbubbles, nbEmptySegments);

    setIsOnSurface();

    return nbubbles;
}

int DelaunayGraphCut::removeDust(int minSegSize)
{
    if(mp->verbose)
        printf("removing dust\n");

    int nbFullSegments = 0;
    StaticVector<int>* fullSegsColor = nullptr;
    segmentFullOrFree(true, &fullSegsColor, nbFullSegments);

    StaticVector<int>* colorsSize = new StaticVector<int>(nbFullSegments);
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

    if(mp->verbose)
        printf("Removed dust cells: %i, Number of segments: %i\n", ndust, nbFullSegments);

    setIsOnSurface();

    return ndust;
}

void DelaunayGraphCut::leaveLargestFullSegmentOnly()
{
    if(mp->verbose)
        printf("Largest full segment only.\n");

    int nsegments;
    StaticVector<int>* colors = nullptr;
    segmentFullOrFree(true, &colors, nsegments);

    StaticVector<int>* colorsSize = new StaticVector<int>(nsegments);
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

    if(mp->verbose)
        printf("Largest full segment only. Done.\n");
}

} // namespace delaunayCut
} // namespace aliceVision
