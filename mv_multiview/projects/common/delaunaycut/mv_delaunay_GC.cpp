#include "mv_delaunay_GC.h"
#include "mv_delaunay_helpers.h"
#include "mv_delaunay_meshSmooth.h"

#include "MaxFlow.hpp"

#include "stdafx.h"

#include "structures/mv_filesio.h"
#include "structures/mv_universe.h"

#include "CUDAInterfaces/refine.h"

#include "hallucinations/hallucinations.h"
#include "mesh/mv_mesh_energy_opt_photo_mem.h"

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

namespace bfs = boost::filesystem;

mv_delaunay_GC::mv_delaunay_GC(multiviewParams* _mp, mv_prematch_cams* _pc)
{
    mp = _mp;
    pc = _pc;

    _camsVertexes.resize(mp->ncams, -1);

    saveTemporaryBinFiles = mp->mip->_ini.get<bool>("largeScale.saveTemporaryBinFiles", false);
    saveTemporaryWrlFiles = mp->mip->_ini.get<bool>("largeScale.saveTemporaryWrlFiles", false);

    GEO::initialize();
    _tetrahedralization = GEO::Delaunay::create(3, "BDEL");
    // _tetrahedralization->set_keeps_infinite(true);
    _tetrahedralization->set_stores_neighbors(true);
    // _tetrahedralization->set_stores_cicl(true);
}

mv_delaunay_GC::~mv_delaunay_GC()
{
}

void mv_delaunay_GC::saveDhInfo(std::string fileNameInfo)
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

    int ncells = getNbCells();
    fwrite(&ncells, sizeof(int), 1, f);
    for(const GC_cellInfo& c: _cellsAttr)
    {
        c.fwriteinfo(f);
    }
    fclose(f);

    if(mp->verbose)
        printf(".");
}

void mv_delaunay_GC::saveDh(std::string fileNameDh, std::string fileNameInfo)
{
    if(mp->verbose)
        printf("Saving triangulation");

    saveDhInfo(fileNameInfo);

    long t1 = clock();
    std::ofstream oFileT(fileNameDh.c_str());
    // oFileT << *_tetrahedralization; // TODO GEOGRAM

    printfElapsedTime(t1);
}

void mv_delaunay_GC::initVertices()
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

void mv_delaunay_GC::computeDelaunay()
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

void mv_delaunay_GC::initCells()
{
    if(mp->verbose)
        printf("initCells ...\n");

    _cellsAttr.resize(_tetrahedralization->nb_cells()); // or nb_finite_cells() if keeps_infinite()

    std::cout << _cellsAttr.size() << " cells created by tetrahedralization." << std::endl;
    for(int i = 0; i < _cellsAttr.size(); ++i)
    {
        GC_cellInfo& c = _cellsAttr[i];

        c.cellId = i;
        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.on = 0.0f;
        c.in = 0.0f;
        c.out = 0.0f;
        c.full = false; // is free
        for(int s = 0; s < 4; ++s)
        {
            c.gEdgeVisWeight[s] = 0.0f; // weights for the 4 faces of the tetrahedron
        }
    }

    if(mp->verbose)
        printf("initCells done\n");
}

void mv_delaunay_GC::displayStatistics()
{
    // Display some statistics

    staticVector<int>* ptsCamsHist = getPtsCamsHist();
    std::cout << "Histogram of number of cams per point:\n";
    for(int i = 0; i < ptsCamsHist->size(); ++i)
        std::cout << "    " << i << ": " << num2str((*ptsCamsHist)[i]) << "\n";
    std::cout << "\n";
    delete ptsCamsHist;

    staticVector<int>* ptsNrcsHist = getPtsNrcHist();
    std::cout << "Histogram of alpha_vis per point:\n";
    for(int i = 0; i < ptsNrcsHist->size(); ++i)
        std::cout << "    " << i << ": " << num2str((*ptsNrcsHist)[i]) << "\n";
    std::cout << "\n";
    delete ptsNrcsHist;
}

void mv_delaunay_GC::loadDhInfo(std::string fileNameInfo, bool doNotCangeFull)
{
    if(mp->verbose)
        printf(".");

    FILE* f = fopen(fileNameInfo.c_str(), "rb");
    int npts = 0;
    fread(&npts, sizeof(int), 1, f);
    _verticesAttr.resize(npts);
    for(GC_vertexInfo& v: _verticesAttr)
    {
        v.freadinfo(f);
    }

    if(mp->verbose)
        printf(".");
    int ncells = 0;
    fread(&ncells, sizeof(int), 1, f);
    _cellsAttr.resize(ncells);
    for(GC_cellInfo& c: _cellsAttr)
    {
        c.freadinfo(f, doNotCangeFull);
    }
    fclose(f);
}

void mv_delaunay_GC::loadDh(std::string fileNameDh, std::string fileNameInfo)
{
    if(mp->verbose)
        printf("Loading triangulation");

    long t1 = clock();
    std::ifstream iFileT(fileNameDh.c_str());
    // iFileT >> *_tetrahedralization;

    loadDhInfo(fileNameInfo, false);

    printfElapsedTime(t1);
}

staticVector<staticVector<int>*>* mv_delaunay_GC::createPtsCams()
{
    long t = std::clock();
    std::cout << "Extract visibilities." << std::endl;
    int npts = getNbVertices();
    staticVector<staticVector<int>*>* out = new staticVector<staticVector<int>*>(npts);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        staticVector<int>* cams = new staticVector<int>(v.getNbCameras());
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

staticVector<int>* mv_delaunay_GC::getPtsCamsHist()
{
    int maxnCams = 0;
    for(const GC_vertexInfo& v: _verticesAttr)
    {
        maxnCams = std::max(maxnCams, (int)v.getNbCameras());
    }
    maxnCams++;
    if(mp->verbose)
        printf("maxnCams %i\n", maxnCams);

    staticVector<int>* ncamsHist = new staticVector<int>(maxnCams);
    ncamsHist->resize_with(maxnCams, 0);

    for(const GC_vertexInfo& v: _verticesAttr)
    {
        (*ncamsHist)[v.getNbCameras()] += 1;
    }

    return ncamsHist;
}

staticVector<int>* mv_delaunay_GC::getPtsNrcHist()
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

    staticVector<int>* nnrcsHist = new staticVector<int>(maxnnrcs);
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

staticVector<int> mv_delaunay_GC::getIsUsedPerCamera() const
{
    long timer = std::clock();

    staticVector<int> cams;
    cams.resize_with(mp->mip->getNbCameras(), 0);

#pragma omp parallel for
    for(int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        const GC_vertexInfo& v = _verticesAttr[vi];
        for(int c = 0; c < v.cams.size(); ++c)
        {
            const int obsCam = v.cams[c];

#pragma OMP_ATOMIC_WRITE
            {
            cams[obsCam] = 1;
            }
        }
    }

    printfElapsedTime(timer, "getIsUsedPerCamera ");
    return cams;
}

staticVector<int> mv_delaunay_GC::getSortedUsedCams() const
{
    const staticVector<int> isUsed = getIsUsedPerCamera();
    staticVector<int> out;
    out.reserve(isUsed.size());
    for(int cameraIndex = 0; cameraIndex < isUsed.size(); ++cameraIndex)
    {
        if(isUsed[cameraIndex] != 0)
            out.push_back(cameraIndex);
    }

    return out;
}

void mv_delaunay_GC::addPointsFromCameraCenters(staticVector<int>* cams, float minDist)
{
    for(int camid = 0; camid < cams->size(); camid++)
    {
        int rc = (*cams)[camid];
        {
            point3d p(mp->CArr[rc].x, mp->CArr[rc].y, mp->CArr[rc].z);

            GEO::index_t vi = locateNearestVertex(p);
            point3d npp;
            if(vi != GEO::NO_VERTEX)
            {
                npp = _verticesCoords[vi];
            }

            if((vi == GEO::NO_VERTEX) || ((npp - mp->CArr[rc]).size() > minDist))
            {
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

void mv_delaunay_GC::addPointsToPreventSingularities(point3d voxel[8], float minDist)
{
    point3d vcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0f;
    point3d extrPts[6];
    point3d fcg;
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
        point3d p(extrPts[i].x, extrPts[i].y, extrPts[i].z);
        GEO::index_t vi = locateNearestVertex(p);
        point3d npp;

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

void mv_delaunay_GC::addHelperPoints(int nGridHelperVolumePointsDim, point3d voxel[8], float minDist)
{
    if(nGridHelperVolumePointsDim <= 0)
        return;

    if(mp->verbose)
        printf("adding helper points ...");

    int ns = nGridHelperVolumePointsDim;
    float md = 1.0f / 500.0f;
    point3d vx = (voxel[1] - voxel[0]);
    point3d vy = (voxel[3] - voxel[0]);
    point3d vz = (voxel[4] - voxel[0]);
    point3d O = voxel[0] + vx * md + vy * md + vz * md;
    vx = vx - vx * 2.0f * md;
    vy = vy - vy * 2.0f * md;
    vz = vz - vz * 2.0f * md;

    float maxSize = 2.0f * (O - voxel[0]).size();
    point3d CG = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0f;

    for(int x = 0; x <= ns; x++)
    {
        for(int y = 0; y <= ns; y++)
        {
            for(int z = 0; z <= ns; z++)
            {
                point3d pt = voxel[0] + vx * ((float)x / (float)ns) + vy * ((float)y / (float)ns) +
                             vz * ((float)z / (float)ns);
                pt = pt + (CG - pt).normalize() * (maxSize * ((float)rand() / (float)RAND_MAX));

                point3d p(pt.x, pt.y, pt.z);
                GEO::index_t vi = locateNearestVertex(p);
                point3d npp;

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


void mv_delaunay_GC::createTetrahedralizationFromDepthMapsCamsVoxel(staticVector<int>* cams,
                                                               staticVector<int>* voxelsIds, point3d voxel[8],
                                                               voxelsGrid* ls)
{
    ///////////////////////////////////////////////////////////////////////////////////////
    printf("Creating delaunay tetrahedralization from depth maps voxel\n");
    long tall = clock();

    bool doFilterOctreeTracks = mp->mip->_ini.get<bool>("largeScale.doFilterOctreeTracks", true);
    int minNumOfConsistentCams = mp->mip->_ini.get<int>("filter.minNumOfConsistentCams", 2);

    ///////////////////////////////////////////////////////////////////////////////////////
    // build tetrahedralization

    int nVertices = 0;

    float minDist = (voxel[0] - voxel[1]).size() / 1000.0f;

    // add points for cam centers
    addPointsFromCameraCenters(cams, minDist);

    // add 6 points to prevent singularities
    addPointsToPreventSingularities(voxel, minDist);

    point3d cgpt;
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
            staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
            staticVector<staticVector<pixel>*>* tracksPointsCams =
                loadArrayOfArraysFromFile<pixel>(fileNameTracksPtsCams);

            long t1 = initEstimate();
            for(int j = 0; j < tracksPoints->size(); j++)
            {
                point3d tp = (*tracksPoints)[j];
                staticVector<pixel>* cams = (*tracksPointsCams)[j];
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
            deleteArrayOfArrays<pixel>(&tracksPointsCams);
        } // if fileexists
    }

    if(mp->verbose)
        printf("voxels tracks triangulated\n");

    /* initialize random seed: */
    srand(time(nullptr));

    int nGridHelperVolumePointsDim = mp->mip->_ini.get<int>("largeScale.nGridHelperVolumePointsDim", 10);
    // add volume points to prevent singularities
    addHelperPoints(nGridHelperVolumePointsDim, voxel, minDist);

    initVertices();

    computeDelaunay();

    printfElapsedTime(tall, "Create Delaunay tetrahedralization from depth map voxels ");

    displayStatistics();
}

void mv_delaunay_GC::computeVerticesSegSize(const std::string& fileNameWrl, bool allPoints, float alpha, bool saveWrl) // allPoints=true, alpha=0
{
    if(mp->verbose)
        printf("creating universe\n");
    int scalePS = mp->mip->_ini.get<int>("global.scalePS", 1);
    int step = mp->mip->_ini.get<int>("global.step", 1);
    float pointToJoinPixSizeDist = (float)mp->mip->_ini.get<double>("delaunaycut.pointToJoinPixSizeDist", 2.0) *
                                   (float)scalePS * (float)step * 2.0f;

    std::vector<pixel> edges;
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
        const point3d& p = _verticesCoords[vi];
        if((v.getNbCameras() > 0) && ((allPoints) || (v.isOnSurface)))
        {
            int rc = v.getCamera(0);

            // go thru all neighbour points
            GEO::vector<VertexIndex> adjVertices;
            _tetrahedralization->get_neighbors(vi, adjVertices);

            for(VertexIndex nvi: adjVertices)
            {
                const GC_vertexInfo& nv = _verticesAttr[nvi];
                const point3d& np = _verticesCoords[nvi];
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

    mv_universe* u = new mv_universe(_verticesAttr.size());

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

    if(saveWrl)
    {
        saveSegsWrl(fileNameWrl);
    }

    delete u;
    if(mp->verbose)
        printf("creating universe done.\n");
}

void mv_delaunay_GC::saveSegsWrl(const std::string& fileNameWrl)
{
    if(mp->verbose)
        std::cout << "saveSegsWrl: " << fileNameWrl << std::endl;
    const std::size_t numvertices = getNbVertices();

    int maxSegId = 0;
    staticVector<int>* segs = new staticVector<int>(numvertices);
    staticVector<point3d>* pts = new staticVector<point3d>(numvertices);

    for(int i = 0; i < _verticesAttr.size(); ++i)
    {
        const GC_vertexInfo& v = _verticesAttr[i];
        if(v.isVirtual())
            continue;

        const point3d& p = _verticesCoords[i];
        int segId = v.segId;
        segs->push_back(segId);
        pts->push_back(p);
        maxSegId = std::max(maxSegId, segId);
    }

    staticVector<voxel>* segscls = new staticVector<voxel>(maxSegId + 1);
    for(int i = 0; i < maxSegId + 1; i++)
    {
        voxel v;
        v.x = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
        v.y = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
        v.z = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
        segscls->push_back(v);
    }

    staticVector<voxel>* cls = new staticVector<voxel>(pts->size());
    for(int i = 0; i < pts->size(); i++)
    {
        voxel v = (*segscls)[(*segs)[i]];
        cls->push_back(v);
    }

    mv_output3D* o3d = new mv_output3D(mp);
    o3d->create_wrl_pts_cls(pts, cls, mp, fileNameWrl);
    delete o3d;
    delete pts;
    delete segscls;
    delete segs;
}


void mv_delaunay_GC::removeSmallSegs(int minSegSize)
{
    if(mp->verbose)
        std::cout << "removeSmallSegs: " << minSegSize << std::endl;
    staticVector<int>* toRemove = new staticVector<int>(getNbVertices());

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

bool mv_delaunay_GC::rayCellIntersection(const point3d& camC, const point3d& p, int tetrahedron, Facet& out_facet,
                                        bool nearestFarest, point3d& out_nlpi) const
{
    out_nlpi = p; // important
    out_facet.cellIndex = -1;
    out_facet.localVertexIndex = -1;

    if(isInfiniteCell(tetrahedron))
    {
        return false;
    }

    point3d linePoint = p;
    point3d lineVect = (camC - p).normalize();
    double mind = (camC - p).size();

    // go thru all triangles
    bool existsTriOnRay = false;
    const point3d* A = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 0)]);
    const point3d* B = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 1)]);
    const point3d* C = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 2)]);
    const point3d* D = &(_verticesCoords[_tetrahedralization->cell_vertex(tetrahedron, 3)]);

    // All the facets of the tetrahedron
    std::array<std::array<const point3d*, 3>, 4> facets {{
        {B, C, D}, // opposite vertex A, index 0
        {A, C, D}, // opposite vertex B, index 1
        {A, B, D}, // opposite vertex C, index 2
        {A, B, C}  // opposite vertex D, index 3
    }};
    int oppositeVertexIndex = -1;
    point3d lpi;
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

mv_delaunay_GC::Facet mv_delaunay_GC::getFacetInFrontVertexOnTheRayToThePoint3D(VertexIndex vi,
                                                                                   point3d& ptt) const
{
    const point3d& p = _verticesCoords[vi];

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
        point3d lpi;
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

mv_delaunay_GC::Facet mv_delaunay_GC::getFacetBehindVertexOnTheRayToTheCam(VertexIndex vi,
                                                                              int cam) const
{
    const point3d& p = _verticesCoords[vi];

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
        point3d lpi;
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

int mv_delaunay_GC::getFirstCellOnTheRayFromCamToThePoint(int cam, point3d& p, point3d& lpi) const
{
    int cam_vi = _camsVertexes[cam];
    point3d camBehind = mp->CArr[cam] + (mp->CArr[cam] - p);
    point3d dir = (p - mp->CArr[cam]).normalize();
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

bool mv_delaunay_GC::isIncidentToType(VertexIndex vi, float type) const
{
    for(int k = 0; true; ++k)
    {
        CellIndex adjCellIndex = vertexToCells(vi, k); // GEOGRAM: set_stores_cicl(true) required
        if(adjCellIndex == GEO::NO_CELL) // last one
            break;
        if(isInfiniteCell(adjCellIndex))
            continue;

        if(_cellsAttr[adjCellIndex].on == type)
            return true;
    }

    return false;
}

bool mv_delaunay_GC::isIncidentToSink(VertexIndex vi, bool sink) const
{
    for(int k = 0; true; ++k)
    {
        CellIndex adjCellIndex = vertexToCells(vi, k); // GEOGRAM: set_stores_cicl(true) required
        if(adjCellIndex == GEO::NO_CELL) // last one
            break;
        if(isInfiniteCell(adjCellIndex))
            continue;

        if(_cellsAttr[adjCellIndex].full == sink)
            return true;
    }

    return false;
}

float mv_delaunay_GC::distFcn(float maxDist, float dist, float distFcnHeight) const
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

double mv_delaunay_GC::facetArea(const Facet &f) const
{
    const point3d& pa = _verticesCoords[getVertexIndex(f, 0)];
    const point3d& pb = _verticesCoords[getVertexIndex(f, 1)];
    const point3d& pc = _verticesCoords[getVertexIndex(f, 2)];

    double a = (pb - pa).size();
    double b = (pc - pa).size();
    double c = (pc - pb).size();
    double p = (a + b + c) / 2.0f;

    double out = std::sqrt(p * (p - a) * (p - b) * (p - c));

    if(std::isnan(out))
        return 0.0;

    return out;
}

double mv_delaunay_GC::getFacetProjectionMaxEdge(Facet& f, int cam) const
{
    const point3d& pa = _verticesCoords[getVertexIndex(f, 0)];
    const point3d& pb = _verticesCoords[getVertexIndex(f, 1)];
    const point3d& pc = _verticesCoords[getVertexIndex(f, 2)];

    point2d pixa, pixb, pixc;
    mp->getPixelFor3DPoint(&pixa, pa, cam);
    mp->getPixelFor3DPoint(&pixb, pb, cam);
    mp->getPixelFor3DPoint(&pixc, pc, cam);

    double a = (pixb - pixa).size();
    double b = (pixc - pixa).size();
    double c = (pixc - pixb).size();

    return std::max(a, std::max(b, c));
}

double mv_delaunay_GC::conj(double val) const
{
    return val;
}

double mv_delaunay_GC::cellMaxEdgeLength(CellIndex ci) const
{
    double dmax = 0.0f;
    for(int i = 0; i < 4; ++i)
    {
        const point3d& pi = _verticesCoords[_tetrahedralization->cell_vertex(ci, i)];
        for(int j = i + 1; j < 4; ++j)
        {
            const point3d& pj = _verticesCoords[_tetrahedralization->cell_vertex(ci, j)];
            double d0 = (pi - pj).size();
            if(std::isnan(d0))
            {
                dmax = std::numeric_limits<double>::max();
            }
            else
            {
                dmax = std::max(dmax, d0);
            }
        }
    }

    if(std::isnan(dmax))
    {
        dmax = std::numeric_limits<double>::max();
    }

    return dmax;
}

double mv_delaunay_GC::cellMinEdgeLength(CellIndex ci)
{
    const point3d& p0 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
    const point3d& p1 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];

    double dmin = (p0 - p1).size();
    for(int i = 0; i < 4; i++)
    {
        const point3d& pi = _verticesCoords[_tetrahedralization->cell_vertex(ci, i)];
        for(int j = i + 1; j < 4; j++)
        {
            const point3d& pj = _verticesCoords[_tetrahedralization->cell_vertex(ci, j)];
            double d0 = (pi - pj).size();
            if(std::isnan(d0))
            {
                dmin = 0.0;
            }
            else
            {
                dmin = std::min(dmin, d0);
            }
        }
    }

    if(std::isnan(dmin))
    {
        dmin = 0.0;
    }

    return dmin;
}

double mv_delaunay_GC::facetMaxEdgeLength(Facet& f) const
{
    double dmax = 0.0;
    const point3d& pa = _verticesCoords[getVertexIndex(f, 0)];
    const point3d& pb = _verticesCoords[getVertexIndex(f, 1)];
    const point3d& pc = _verticesCoords[getVertexIndex(f, 2)];
    double d0 = (pa - pb).size();
    double d1 = (pa - pc).size();
    double d2 = (pc - pb).size();
    dmax = std::max(dmax, d0);
    dmax = std::max(dmax, d1);
    dmax = std::max(dmax, d2);

    return dmax;
}

double mv_delaunay_GC::maxEdgeLength() const
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

double mv_delaunay_GC::averageEdgeLength() const
{
    double d = 0.0;
    double n = 0.0;

    for(int ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(int k = 0; k < 4; ++k)
        {
            Facet f(ci, k);
            const point3d& pa = _verticesCoords[getVertexIndex(f, 0)];
            const point3d& pb = _verticesCoords[getVertexIndex(f, 1)];
            const point3d& pc = _verticesCoords[getVertexIndex(f, 2)];
            d += (pa - pb).size();
            d += (pa - pc).size();
            d += (pc - pb).size();
            n += 3.0f;
        }
    }
    return d / n;
}

point3d mv_delaunay_GC::cellCircumScribedSphereCentre(CellIndex ci) const
{
    // http://www.mps.mpg.de/homes/daly/CSDS/t4h/tetra.htm

    const point3d r0 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
    const point3d r1 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
    const point3d r2 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
    const point3d r3 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];

    const point3d d1 = r1 - r0;
    const point3d d2 = r2 - r0;
    const point3d d3 = r3 - r0;

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

    return r0 + point3d(x, y, z);
}

double mv_delaunay_GC::cellVolume(CellIndex ci) const
{
    // http://en.wikipedia.org/wiki/Tetrahedron#Volume

    const point3d a = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
    const point3d b = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
    const point3d c = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
    const point3d d = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];

    return fabs(dot(a - d, cross(b - d, c - d))) / 6.0;
}

point3d mv_delaunay_GC::cellCentreOfGravity(CellIndex ci) const
{
    const point3d r0 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
    const point3d r1 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
    const point3d r2 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
    const point3d r3 = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];
    return (r0 + r1 + r2 + r3) / 4.0;
}

// Returns a small score if one of the tetrahedon (from one side of the facet) is strange (the point in front of the current facet is far from the center).
// Returns the best value when the tetrahedons on both side of the facet are equilaterals.
double mv_delaunay_GC::getFaceWeight(const Facet& f1) const
{
    const Facet f2 = mirrorFacet(f1);
    const point3d s1 = cellCircumScribedSphereCentre(f1.cellIndex);
    const point3d s2 = cellCircumScribedSphereCentre(f2.cellIndex);

    const point3d A = _verticesCoords[getVertexIndex(f1, 0)];
    const point3d B = _verticesCoords[getVertexIndex(f1, 1)];
    const point3d C = _verticesCoords[getVertexIndex(f1, 2)];

    const point3d n = cross((B - A).normalize(), (C - A).normalize()).normalize();

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

float mv_delaunay_GC::weightFromSim(float sim)
{
    float ssim = std::min(1.0f, std::max(-1.0f, sim));
    // sim = [-1:0.1:1]; weight = 1-exp(-20*((sim+1)/2)); plot(sim,weight,'r-');
    return (1.0f - std::exp(-20.0f * ((ssim + 1.0f) / 2.0f)));
}

float mv_delaunay_GC::weightFcn(float nrc, bool labatutWeights, int  /*ncams*/)
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

bool mv_delaunay_GC::isCellSmallForPoint(CellIndex ci, VertexIndex vi) const
{
    const GC_vertexInfo& a = _verticesAttr[_tetrahedralization->cell_vertex(ci, 0)];
    const GC_vertexInfo& b = _verticesAttr[_tetrahedralization->cell_vertex(ci, 1)];
    const GC_vertexInfo& c = _verticesAttr[_tetrahedralization->cell_vertex(ci, 2)];
    const GC_vertexInfo& d = _verticesAttr[_tetrahedralization->cell_vertex(ci, 3)];

    const point3d& pa = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];

    double chMaxPixSize = std::max(a.pixSize, std::max(b.pixSize, std::max(c.pixSize, d.pixSize)));
    double chSize = (cellCircumScribedSphereCentre(ci) - pa).size();

    return ((5.0 * chMaxPixSize > chSize) && (2.0 * chMaxPixSize < _verticesAttr[vi].pixSize));
}

void mv_delaunay_GC::fillGraph(bool fixesSigma, float nPixelSizeBehind, bool allPoints, bool behind,
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
        c.color = 0;
        for(int s = 0; s < 4; s++)
        {
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }

    // choose random order to prevent waiting
    staticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

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

void mv_delaunay_GC::fillGraphPartPtRc(int& out_nstepsFront, int& out_nstepsBehind, int vertexIndex, int cam,
                                       float weight, bool fixesSigma, float nPixelSizeBehind, bool allPoints,
                                       bool behind, bool fillOut, float distFcnHeight)  // fixesSigma=true nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 fillOut=1 distFcnHeight=0
{
    out_nstepsFront = 0;
    out_nstepsBehind = 0;

    int maxint = 1000000.0f; // std::numeric_limits<int>::std::max()

    const point3d& po = _verticesCoords[vertexIndex];
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

        point3d p = po;
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

            point3d pold = p;
            Facet f1, f2;
            point3d lpi;
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

        point3d p = po; // HAS TO BE HERE !!!

        bool ok = (ci != GEO::NO_CELL) && ((allPoints) || (_cellsAttr[ci].full));
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

            point3d pold = p;
            point3d lpi;
            // find cell which is farest to the cam and which intersect cam-p ray
            if((!farestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, ci, f1, f2, lpi)) ||
               ((po - pold).size() >= maxDist) || ((!allPoints) && (!c.full)))
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

void mv_delaunay_GC::forceTedgesByGradientCVPR11(bool fixesSigma, float nPixelSizeBehind)
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
    staticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

#pragma omp parallel for
    for(int i = 0; i < vetexesToProcessIdsRand->size(); ++i)
    {
        int vi = (*vetexesToProcessIdsRand)[i];
        GC_vertexInfo& v = _verticesAttr[vi];
        if(v.isVirtual())
            continue;

        const point3d& po = _verticesCoords[vi];
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
                point3d p = po; // HAS TO BE HERE !!!
                float maxDist = nPixelSizeBehind * mp->getCamPixelSize(p, cam);
                if(fixesSigma)
                {
                    maxDist = nPixelSizeBehind;
                }

                bool ok = (ci != GEO::NO_CELL);
                while(ok)
                {
                    point3d pold = p;
                    point3d lpi;
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

void mv_delaunay_GC::forceTedgesByGradientIJCV(bool fixesSigma, float nPixelSizeBehind)
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
    staticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

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

        const point3d& po = _verticesCoords[vi];
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
                point3d p = po; // HAS TO BE HERE !!!
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
                    point3d lpi;
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
                point3d p = po; // HAS TO BE HERE !!!
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
                    point3d lpi;
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

void mv_delaunay_GC::filterPointsWithHigherPixelSize(bool fixesSigma, float nPixelSizeBehind)
{
    if(mp->verbose)
        printf("Filtering points with higher pixelSize\n");
    long t2 = clock();

    // choose random order to prevent waiting
    staticVector<int>* vetexesToProcessIdsRand = createRandomArrayOfIntegers(_verticesAttr.size());

    int64_t avStepsFront = 0;
    int64_t aAvStepsFront = 0;
    int64_t avStepsBehind = 0;
    int64_t nAvStepsBehind = 0;

#pragma omp parallel for reduction(+:avStepsFront,aAvStepsFront,avStepsBehind,nAvStepsBehind)
    for(int i = 0; i < vetexesToProcessIdsRand->size(); i++)
    {
        int vi = (*vetexesToProcessIdsRand)[i];
        GC_vertexInfo& v = _verticesAttr[vi];
        if(v.isVirtual() || v.nrc == 0)
            continue;

        const point3d& po = _verticesCoords[vi];
        int c = mp->getCamsMinPixelSizeIndex(po, v.cams);
        // for (int c=0;c<cams->size();c++)
        {
            int nstepsFront = 0;
            int nstepsBehind = 0;

            int cam = v.cams[c];
            bool erasePtNrc = false;

            float minPixSize = v.pixSize;
            float maxDist = 0.0f;
            if(fixesSigma)
            {
                maxDist = nPixelSizeBehind;
            }
            else
            {
                maxDist = nPixelSizeBehind * mp->getCamPixelSize(po, cam);
            }

            {
                CellIndex ci = getFacetInFrontVertexOnTheRayToTheCam(vi, cam).cellIndex;
                point3d p = po; // HAS TO BE HERE !!!
                bool ok = (ci != GEO::NO_CELL);
                while(ok)
                {
                    ++nstepsFront;

                    for(int k = 0; k < 4; ++k)
                    {
                        const VertexIndex ki = _tetrahedralization->cell_vertex(ci, k);
                        const GC_vertexInfo& kVertexAttr = _verticesAttr[ki];
                        const point3d& kVertex = _verticesCoords[ki];

                        if(((kVertex - po).size() < maxDist) &&
                           (kVertexAttr.isReal()) && (kVertexAttr.pixSize < minPixSize))
                        {
                            const int mintcam = kVertexAttr.cams[mp->getCamsMinPixelSizeIndex(kVertex, kVertexAttr.cams)];
                            if((cam != mintcam) && (checkCamPairAngle(cam, mintcam, mp, 0.0f, 50.0f)))
                            {
                                const double d = pointLineDistance3D(po, kVertex,
                                                               mp->CArr[mintcam] - kVertex.normalize());
                                if(d < mp->getCamPixelSize(kVertex, mintcam))
                                {
                                    erasePtNrc = true;
                                }
                            }
                        }
                    }

                    Facet f1, f2;
                    point3d lpi;
                    // find cell which is nearest to the cam and which is
                    // intersected with cam-p ray
                    if(((p - po).size() > maxDist) ||
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
                CellIndex ci = getFacetBehindVertexOnTheRayToTheCam(vi, cam).cellIndex;
                point3d p = po; // HAS TO BE HERE !!!
                bool ok = (ci != GEO::NO_CELL);
                while(ok)
                {
                    ++nstepsBehind;

                    for(int k = 0; k < 4; ++k)
                    {
                        const VertexIndex ki = _tetrahedralization->cell_vertex(ci, k);
                        const GC_vertexInfo& kVertexAttr = _verticesAttr[ki];
                        const point3d& kVertex = _verticesCoords[ki];

                        if(((kVertex - po).size() < maxDist) &&
                           kVertexAttr.isReal() && (kVertexAttr.pixSize < minPixSize))
                        {
                            if(cam != kVertexAttr.cams[mp->getCamsMinPixelSizeIndex(kVertex, kVertexAttr.cams)])
                            {
                                erasePtNrc = true;
                            }
                        }
                    }

                    Facet f1, f2;
                    point3d lpi;
                    // find cell which is farest to the cam and which is
                    // intersected with cam-p
                    // ray
                    if(((p - po).size() > maxDist) ||
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
            }

            if(erasePtNrc)
            {
#pragma OMP_ATOMIC_WRITE
                v.nrc = 0;
            }

            avStepsFront += nstepsFront;
            aAvStepsFront += 1;
            avStepsBehind += nstepsBehind;
            nAvStepsBehind += 1;
        }
    }

    delete vetexesToProcessIdsRand;

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

void mv_delaunay_GC::updateGraphFromTmpPtsCamsHexah(staticVector<int>* incams, point3d hexah[8],
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

void mv_delaunay_GC::updateGraphFromTmpPtsCamsHexahRC(int rc, point3d hexah[8], std::string tmpCamsPtsFolderName,
                                                      bool labatutWeights, float  /*distFcnHeight*/)
{

    // fill edges
    int nin = 0;
    int nout = 0;
    int cnull = 0;
    int nwup = 0;

    std::string camPtsFileName;
    camPtsFileName = tmpCamsPtsFolderName + "camPtsGrid_" + num2strFourDecimal(rc) + ".bin";

    bool doFilterOctreeTracks = mp->mip->_ini.get<bool>("largeScale.doFilterOctreeTracks", true);
    int minNumOfConsistentCams = mp->mip->_ini.get<int>("filter.minNumOfConsistentCams", 2);

    /////////////////////////////////////////////////////////////////////////////////
    FILE* f = fopen(camPtsFileName.c_str(), "rb");
    while(feof(f) == 0)
    {
        GC_camVertexInfo pnt;
        pnt.freadinfo(f);
        // printf("%f, %i, %f, %f,
        // %f\n",pnt.sim,pnt.nrc,pnt.point.x,pnt.point.y,pnt.point.z);
        point3d pt = pnt.point;

        float weight = weightFcn((float)pnt.nrc, labatutWeights, (float)pnt.ncams);

        nout++;

        staticVector<point3d>* lshi = lineSegmentHexahedronIntersection(pt, mp->CArr[rc], hexah);
        if((!isPointInHexahedron(pt, hexah)) && (lshi->size() >= 1) && (feof(f) == 0) &&
           ((!doFilterOctreeTracks) || ((doFilterOctreeTracks) && (pnt.ncams >= minNumOfConsistentCams))))
        {
            nin++;
            point3d lpi = pt;
            point3d camBehind = mp->CArr[rc] + (mp->CArr[rc] - pt);
            CellIndex ci = getFirstCellOnTheRayFromCamToThePoint(rc, pt, lpi);
            if(ci != GEO::NO_CELL)
            {
                // update weights on the sp-cam half line
                CellIndex tmp_ci = ci;
                point3d p = mp->CArr[rc];
                // _cellsAttr[ci].cellSWeight = (float)maxint;

                bool ok = tmp_ci != GEO::NO_CELL;

                while(ok)
                {
                    {
#pragma OMP_ATOMIC_UPDATE
                        _cellsAttr[tmp_ci].out += weight;
                    }

                    Facet f1, f2;
                    point3d lpi;
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

int mv_delaunay_GC::setIsOnSurface()
{
    // set is on surface
    for(GC_vertexInfo& v: _verticesAttr)
    {
        v.isOnSurface = false;
    }

    int nbSurfaceFacets = 0;
    // loop over all facets
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellsAttr[f1.cellIndex].full; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellsAttr[f2.cellIndex].full; // get if it is occupied

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

bool mv_delaunay_GC::isCamInFrontOfFacet(Facet f, int rc)
{
    // point3d pa =
    // convertPointToPoint3d(f.first->vertex((f.second+1)%4)->point());
    // point3d pb =
    // convertPointToPoint3d(f.first->vertex((f.second+2)%4)->point());
    // point3d pc =
    // convertPointToPoint3d(f.first->vertex((f.second+3)%4)->point());
    // point3d pd = convertPointToPoint3d(f.first->vertex(f.second)->point());
    const point3d pa = _verticesCoords[getVertexIndex(f, 0)];
    const point3d pb = _verticesCoords[getVertexIndex(f, 1)];
    const point3d pc = _verticesCoords[getVertexIndex(f, 2)];
    const point3d pd = _verticesCoords[getOppositeVertexIndex(f)];

    const point3d v1 = (pb - pa).normalize();
    const point3d v2 = (pc - pa).normalize();
    point3d n = cross(v1, v2).normalize();

    double d1 = orientedPointPlaneDistance(pd, pa, n);

    if(d1 < 0.0)
    {
        n.x = -n.x;
        n.y = -n.y;
        n.z = -n.z;
    }

    // IMPORTANT we measure angle(NORMAL, ray to the camera) ... the angle is from 0 to 180
    return (angleBetwV1andV2(n, (mp->CArr[rc] - pa).normalize()) < 90.0);
}

float mv_delaunay_GC::triangle_area(mv2DTriangle& t)
{
    // float a = (pb-pa).size();
    // float b = (pc-pa).size();
    // float c = (pc-pb).size();
    float a = (t.pts[1] - t.pts[0]).size();
    float b = (t.pts[2] - t.pts[0]).size();
    float c = (t.pts[2] - t.pts[1]).size();
    float p = (a + b + c) / 2.0f;

    return sqrt(p * (p - a) * (p - b) * (p - c));
}

float mv_delaunay_GC::triangle_maxSide(mv2DTriangle& t)
{
    // float a = (pb-pa).size();
    // float b = (pc-pa).size();
    // float c = (pc-pb).size();
    float a = (t.pts[1] - t.pts[0]).size();
    float b = (t.pts[2] - t.pts[0]).size();
    float c = (t.pts[2] - t.pts[1]).size();

    return std::max(a, std::max(b, c));
}

float mv_delaunay_GC::triangle_minSide(mv2DTriangle& t)
{
    // float a = (pb-pa).size();
    // float b = (pc-pa).size();
    // float c = (pc-pb).size();
    float a = (t.pts[1] - t.pts[0]).size();
    float b = (t.pts[2] - t.pts[0]).size();
    float c = (t.pts[2] - t.pts[1]).size();

    return std::min(a, std::min(b, c));
}

float mv_delaunay_GC::triangle_incircle_area(mv2DTriangle& t)
{
    // float a = (pb-pa).size();
    // float b = (pc-pa).size();
    // float c = (pc-pb).size();
    float a = (t.pts[1] - t.pts[0]).size();
    float b = (t.pts[2] - t.pts[0]).size();
    float c = (t.pts[2] - t.pts[1]).size();
    float p = (a + b + c) / 2.0f;

    float r = sqrt(((p - a) * (p - b) * (p - c)) / p);

    return (float)M_PI * r * r;
}

float mv_delaunay_GC::triangle_circumscribed_area(mv2DTriangle& t)
{
    // float a = (pb-pa).size();
    // float b = (pc-pa).size();
    // float c = (pc-pb).size();
    float a = (t.pts[1] - t.pts[0]).size();
    float b = (t.pts[2] - t.pts[0]).size();
    float c = (t.pts[2] - t.pts[1]).size();
    float p = (a + b + c) / 2.0f;

    float r = ((a * b * c) / (2.0f * sqrt(p * (p - a) * (p - b) * (p - c)))) / 2.0f;

    return (float)M_PI * r * r;
}

void mv_delaunay_GC::saveMaxflowToWrl(std::string dirName, std::string fileNameTxt, std::string fileNameTxtCam,
                                      std::string fileNameWrl, std::string fileNameWrlTex, std::string fileNamePly,
                                      int camerasPerOneOmni)
{
    staticVector<int>* cams = new staticVector<int>(mp->ncams);
    for(int i = 0; i < mp->ncams; i++)
    {
        cams->push_back(i);
    }
    saveMaxflowToWrl(dirName, fileNameTxt, fileNameTxtCam, fileNameWrl, fileNameWrlTex, fileNamePly, camerasPerOneOmni,
                     cams);
    delete cams;
}

bool mv_delaunay_GC::hasVertex(CellIndex ci, VertexIndex vi) const
{
    return (_tetrahedralization->cell_vertex(ci, 0) == vi ||
            _tetrahedralization->cell_vertex(ci, 1) == vi ||
            _tetrahedralization->cell_vertex(ci, 2) == vi ||
            _tetrahedralization->cell_vertex(ci, 3) == vi);
}

void mv_delaunay_GC::getIncidentCellsToCellAndVertexOfTheCellIndexes(int vIncident[3], CellIndex ci,
                                                                     VertexIndex vi) const
{
    int id = 0;
    for(int k = 0; k < 4; ++k)
    {
        const CellIndex nci = _tetrahedralization->cell_adjacent(ci, k);
        if(nci == GEO::NO_CELL)
            continue;
        if(hasVertex(nci, vi))
        {
            assert(id < 3);
            vIncident[id] = k;
            ++id;
        }
    }
}

void mv_delaunay_GC::getIncidentCellsToCellAndEdgeOfTheCellIndexes(int vIncident[2], CellIndex ci, int lvi, int lvj) const
{
    VertexIndex v1 = _tetrahedralization->cell_vertex(ci, lvi);
    VertexIndex v2 = _tetrahedralization->cell_vertex(ci, lvj);

    int id = 0;
    for(int k = 0; k < 4; ++k)
    {
        const CellIndex nci = _tetrahedralization->cell_adjacent(ci, k);
        if(nci == GEO::NO_CELL)
            continue;
        if((hasVertex(nci, v1)) && (hasVertex(nci, v2)))
        {
            assert(id < 3);
            vIncident[id] = k;
            ++id;
        }
    }
}

int mv_delaunay_GC::erosionDilatation(bool sink)
{
    if(mp->verbose)
        printf("erosionDilatation\n");

    int nmorphed = 0;

    staticVector<CellIndex> toDoInverse(_cellsAttr.size());

    // standalone tetrahedrons
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        bool doInverse = false;

        const CellIndex nci0 = _tetrahedralization->cell_adjacent(ci, 0);
        const CellIndex nci1 = _tetrahedralization->cell_adjacent(ci, 1);
        const CellIndex nci2 = _tetrahedralization->cell_adjacent(ci, 2);
        const CellIndex nci3 = _tetrahedralization->cell_adjacent(ci, 3);
        if(((c.full == sink) && (_cellsAttr[nci0].full == !sink)) ||
           (_cellsAttr[nci1].full == !sink) || (_cellsAttr[nci2].full == !sink) || (_cellsAttr[nci3].full == !sink))
        {
            doInverse = true;
            ++nmorphed;
        }

        if(doInverse)
            toDoInverse.push_back(ci);
    }

    for(std::size_t i = 0; i < toDoInverse.size(); ++i)
    {
        CellIndex ci = toDoInverse[i];
        GC_cellInfo& c = _cellsAttr[ci];
        c.full = !c.full;
    }

    if(mp->verbose)
        printf("erosionDilatation : %i\n", nmorphed);

    return nmorphed;
}

void mv_delaunay_GC::graphCutPostProcessing()
{
    long timer = std::clock();
    std::cout << "Graph cut post-processing." << std::endl;
    invertFullStatusForSmallLabels();

    staticVector<CellIndex> toDoInverse;
    toDoInverse.reserve(getNbCells());

    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        int count = 0;
        for(int k = 0; k < 4; ++k)
        {
            const CellIndex nci = _tetrahedralization->cell_adjacent(ci, k);
            if(nci == GEO::NO_CELL)
                continue;
            count += (_cellsAttr[nci].full != c.full);
        }
        if(count > 2)
            toDoInverse.push_back(ci);
    }
    for(std::size_t i = 0; i < toDoInverse.size(); ++i)
    {
        CellIndex ci = toDoInverse[i];
        GC_cellInfo& c = _cellsAttr[ci];
        c.full = !c.full;
    }
    std::cout << "Graph cut post-processing done." << std::endl;

    printfElapsedTime(timer, "Graph cut post-processing ");
}

float mv_delaunay_GC::getAveragePixelSize() const
{
    // compute average pixel size
    float avPixelSize = 0.0f;
    float navPixelSize = 0.0f;
    for(VertexIndex i = 0; i < _verticesAttr.size(); ++i)
    {
        const GC_vertexInfo& v = _verticesAttr[i];
        const point3d& p = _verticesCoords[i];
        for(int c = 0; c < v.cams.size(); ++c)
        {
            avPixelSize += mp->getCamPixelSize(p, v.cams[c]);
            navPixelSize += 1.0f;
        }
    }
    avPixelSize /= navPixelSize;

    return avPixelSize;
}

void mv_delaunay_GC::freeUnwantedFullCells(std::string  /*folderName*/, point3d* hexah)
{
    setIsOnSurface();

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

            GC_cellInfo& c = _cellsAttr[adjCellIndex];
            c.full = false;
        }
    }

    // remove cells that have a point outside hexahedron
    if(hexah != nullptr)
    {
        int nremoved = 0;
        point3d hexahinf[8];
        inflateHexahedron(hexah, hexahinf, 1.001);
        for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
        {
            GC_cellInfo& c = _cellsAttr[ci];
            if(isInfiniteCell(ci) || !c.full)
                continue;

            const point3d& pa = _verticesCoords[_tetrahedralization->cell_vertex(ci, 0)];
            const point3d& pb = _verticesCoords[_tetrahedralization->cell_vertex(ci, 1)];
            const point3d& pc = _verticesCoords[_tetrahedralization->cell_vertex(ci, 2)];
            const point3d& pd = _verticesCoords[_tetrahedralization->cell_vertex(ci, 3)];

            if((!isPointInHexahedron(pa, hexahinf)) ||
               (!isPointInHexahedron(pb, hexahinf)) ||
               (!isPointInHexahedron(pc, hexahinf)) ||
               (!isPointInHexahedron(pd, hexahinf)))
            {
                c.full = false;
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

void mv_delaunay_GC::saveMaxflowToWrl(std::string  /*dirName*/, std::string fileNameTxt, std::string  /*fileNameTxtCam*/,
                                      std::string fileNameWrl, std::string  /*fileNameWrlTex*/, std::string  /*fileNamePly*/,
                                      int camerasPerOneOmni, staticVector<int>* cams)
{
    if(mp->verbose)
        printf("saving mincut to wrl\n");

    // important because it changes the in out
    setIsOnSurface();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    // float avOutEnergy = computeAverageOutEnergyOnTheSurface()/100.0f;
    // float avOutEnergy = computeAverageOutEnergy()/100.0f;

    // read the solution:
    FILE* f = fopen(fileNameTxt.c_str(), "w");
    // FILE *frc = fopen(fileNameTxtCam.c_str(),"w");

    fprintf(f, "%i\n", 3);

    fprintf(f, "%i\n", (int)getNbVertices());
    for(const point3d& p: _verticesCoords)
    {
        fprintf(f, "%f %f %f\n", p.x, p.y, p.z);
    }

    // loop over all facets
    int nttt = 0;
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellsAttr[f1.cellIndex].full; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            bool vo = _cellsAttr[f2.cellIndex].full; // get if it is occupied

            if(uo != vo)
            {
                ++nttt;

                assert(!(isInfiniteCell(f1.cellIndex) && isInfiniteCell(f2.cellIndex)));
            }
        }
    }
    fprintf(f, "%i\n", nttt);

    // loop over all facets
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellsAttr[f1.cellIndex].full; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellsAttr[f2.cellIndex].full; // get if it is occupied

            if(uo == vo)
                continue;

            VertexIndex vertices[3];
            vertices[0] = getVertexIndex(f1, 0);
            vertices[1] = getVertexIndex(f1, 1);
            vertices[2] = getVertexIndex(f1, 2);

            /*
            float s = std::min(f1.first->info().in/avOutEnergy,1.0f);
            rgb c = getColorFromJetColorMap(s);

            c.r = 255; c.g = 255; c.b = 255;

            if (f1.first->info().on==0.0f) {
                    c.r = 0; c.g = 0; c.b = 255;
            };

            if (f1.first->info().on==2.0f) {
                    c.r = 0; c.g = 255; c.b = 0;
            };

            if (f1.first->info().on==3.0f) {
                    c.r = 255; c.g = 0; c.b = 0;
            };
            */

            /*
            rgb c;
            if (isVisibleFacet(f1, camsMap, color)==true) {
                    c.r = 0; c.g = 255; c.b = 0;
            }else{
                    c.r = 255; c.g = 0; c.b = 0;
            };
            trisColors->push_back(c);
            */

            point3d points[3];
            for(int k = 0; k < 3; ++k)
            {
                points[k] = _verticesCoords[vertices[k]];
            }
            point3d oax = (points[0] + points[1] + points[2]) / 3.0;
            point3d VV[3];
            for(int k = 0; k < 3; ++k)
            {
                VV[k] = (points[k] - oax).normalize();
            }

            point3d D = _verticesCoords[getOppositeVertexIndex(f1)];
            point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

            if(orientedPointPlaneDistance(D, oax, N) < 0.0)
            {
                N.x = -N.x;
                N.y = -N.y;
                N.z = -N.z;
            }

            if(signedAngleBetwV1andV2(VV[0], VV[1], N) > signedAngleBetwV1andV2(VV[0], VV[2], N))
            {
                fprintf(f, "%i %i %i\n", vertices[0], vertices[1], vertices[2]);
                // int cam = getBestCameraForColoring(f1);
                // fprintf(frc,"%i\n",cam);
            }
            else
            {
                fprintf(f, "%i %i %i\n", vertices[0], vertices[2], vertices[1]);
                // int cam = getBestCameraForColoring(f2);
                // fprintf(frc,"%i\n",cam);
            }
        }
    }

    // delete camsMap;

    fclose(f);
    // fclose(frc);

    if(mp->verbose)
        printf("n of maxflow is %i\n", nttt);

    mv_output3D* o3d = new mv_output3D(mp);

    if(saveTemporaryWrlFiles)
    {
        o3d->writeCamerasToWrl(cams, fileNameWrl + "_cams.wrl", mp, camerasPerOneOmni, 0.0f);
        o3d->inline2Wrls(fileNameWrl + "_meshCams.wrl", fileNameWrl, fileNameWrl + "_cams.wrl");
        o3d->save_triangulation_to_wrl(mp, fileNameTxt, fileNameWrl);
    }

    // o3d->create_wrl_for_delaunay_cut(mp, fileNameTxt, fileNameTxtCam,
    // fileNameWrlTex, dirName,
    // camerasPerOneOmni, cams);
    // o3d->create_ply_for_delaunay_cut_strecha(mp, fileNameTxt, fileNamePly);
    mv_mesh* me = new mv_mesh();
    me->loadFromTxt(fileNameTxt);

    // std::string fileNameWrl1 = fileNameWrl + "_clored.wrl";
    // o3d->saveMvMeshToWrl(me, fileNameWrl1.c_str(), trisColors);

    delete me;

    delete o3d;

    // delete trisColors;

    if(mp->verbose)
        printf("done\n");
}

void mv_delaunay_GC::segmentCells()
{
    if(mp->verbose)
        printf("segmenting cells\n");

    int n = getNbCells(); // number of tetrahedrons

    staticVector<int>* colors = new staticVector<int>(n);
    colors->resize_with(n, -1);

    staticVector<sortedId>* ncolors = new staticVector<sortedId>(n);
    ncolors->resize(n);
    for(int i = 0; i < n; i++)
    {
        (*ncolors)[i].id = i;
        (*ncolors)[i].value = 0.0;
    }

    staticVector<CellIndex>* buff = new staticVector<CellIndex>(n);

    int col = 0;

    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        assert(c.cellId > -1);

        if(((*colors)[c.cellId] == -1) && (c.full))
        {
            // backtrack all connected interior cells
            buff->resize(0);
            buff->push_back(ci);

            (*colors)[c.cellId] = col;
            (*ncolors)[col].value += 1.0;

            while(buff->size() > 0)
            {
                CellIndex tmp_ci = buff->pop();

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    const GC_cellInfo& nc = _cellsAttr[nci];
                    if((nc.cellId > -1) && ((*colors)[nc.cellId] == -1) && (nc.full))
                    {
                        (*colors)[nc.cellId] = col;
                        (*ncolors)[col].value += 1.0;
                        buff->push_back(nci);
                    }
                }
            }
            ++col;
        }
    }

    qsort(&(*ncolors)[0], col, sizeof(sortedId), qsortCompareSortedIdDesc);
    int bestcolor = (*ncolors)[0].id;

    if(mp->verbose)
        printf("all %i, first best %f, second best %f, colors %i\n", n, (*ncolors)[0].value,
               (*ncolors)[std::max(col - 1, 1)].value, col);

    // TODO FACA: same as leaveLargestFullSegmentOnly?
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        assert(c.cellId > -1);

        c.full = ((*colors)[c.cellId] == bestcolor);
    }

    delete ncolors;
    delete colors;
}

void mv_delaunay_GC::invertFullStatusForSmallLabels()
{
    if(mp->verbose)
        printf("filling small holes\n");

    const int n = getNbCells();
    staticVector<int>* colors = new staticVector<int>(n);
    colors->resize_with(n, -1);

    staticVector<sortedId>* ncolors = new staticVector<sortedId>(n);
    ncolors->resize(n);
    for(int i = 0; i < n; ++i)
    {
        (*ncolors)[i].id = i;
        (*ncolors)[i].value = 0.0;
    }

    staticVector<CellIndex>* buff = new staticVector<CellIndex>(n);

    int col = 0;

    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        assert(c.cellId > -1);

        if((*colors)[c.cellId] == -1)
        {
            // backtrack all connected interior cells
            buff->resize(0);
            buff->push_back(ci);

            (*colors)[c.cellId] = col;
            (*ncolors)[col].value += 1.0;

            while(buff->size() > 0)
            {
                CellIndex tmp_ci = buff->pop();

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    const GC_cellInfo& nc = _cellsAttr[nci];
                    if((nc.cellId > -1) && ((*colors)[nc.cellId] == -1) && (nc.full == c.full))
                    {
                        (*colors)[nc.cellId] = col;
                        (*ncolors)[col].value += 1.0;
                        buff->push_back(nci);
                    }
                }
            }
            ++col;
        }
    }

    int nfilled = 0;
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        assert(c.cellId > -1);

        int col = (*colors)[c.cellId];
        if((*ncolors)[col].value < 100.0f)
        {
            c.full = !c.full;
            ++nfilled;
        }
    }

    if(mp->verbose)
        printf("all %i, colors %i, filled %i\n", n, col, nfilled);

    delete ncolors;
    delete colors;
}

void mv_delaunay_GC::reconstructVoxel(point3d hexah[8], staticVector<int>* voxelsIds, std::string folderName,
                                      std::string tmpCamsPtsFolderName, bool removeSmallSegments,
                                      staticVector<point3d>* hexahsToExcludeFromResultingMesh, voxelsGrid* ls,
                                      point3d spaceSteps)
{
    staticVector<int>* cams = pc->findCamsWhichIntersectsHexahedron(hexah);

    if(cams->size() < 1)
        throw std::logic_error("No camera to make the reconstruction");

    int camerasPerOneOmni = 1;
    std::string fileNameDh = folderName + "delaunayTriangulation.bin";
    std::string fileNameInfo = folderName + "delaunayTriangulationInfo.bin";
    std::string fileNameStGraph = folderName + "stGraph.bin";
    std::string fileNameStSolution = folderName + "stGraphSolution.bin";
    std::string fileNameTxt = folderName + "delaunayTrianglesMaxflow.txt";
    std::string fileNameTxtCam = folderName + "delaunayTrianglesCamerasForColoringMaxflow.txt";
    std::string fileNameDelanuayVerticesSegWrl = folderName + "delaunayVerticesSeg.wrl";
    std::string fileNameDelanuayVerticesSegFilteredWrl = folderName + "delaunayVerticesSegFiltered.wrl";

    // Load tracks and create tetrahedralization (into T variable)
    createTetrahedralizationFromDepthMapsCamsVoxel(cams, voxelsIds, hexah, ls);

    //float nPixelSizeBehind = (float)mp->mip->_ini.get<double>("delaunaycut.filterPtsWithHigherPixSizeInDist", 5.0f)*spaceSteps.size();
    //filterPointsWithHigherPixelSize(true, nPixelSizeBehind);
    // initTriangulationDefaults(folderName + "delaunayVerticesFiltered.wrl");

    computeVerticesSegSize(fileNameDelanuayVerticesSegWrl, true, 0.0f, saveTemporaryWrlFiles);
    if(removeSmallSegments) // false
    {
        removeSmallSegs(2500); // TODO FACA: to decide
        saveSegsWrl(fileNameDelanuayVerticesSegFilteredWrl);
    }

    bool updateLSC = mp->mip->_ini.get<bool>("largeScale.updateLSC", true);

    reconstructExpetiments(cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt, fileNameTxtCam,
                           camerasPerOneOmni, updateLSC, hexah, tmpCamsPtsFolderName, hexahsToExcludeFromResultingMesh,
                           spaceSteps);

    bool saveOrNot = mp->mip->_ini.get<bool>("largeScale.saveDelaunayTriangulation", false);
    if(saveOrNot)
    {
        saveDh(fileNameDh, fileNameInfo);
    }

    // loadDh(fileNameDh, fileNameInfo);
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

void mv_delaunay_GC::addToInfiniteSw(float sW)
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

void mv_delaunay_GC::reconstructGC(float alphaQual, std::string baseName, staticVector<int>* cams,
                                   std::string folderName, std::string fileNameStGraph, std::string fileNameStSolution,
                                   std::string fileNameTxt, std::string fileNameTxtCam, int camerasPerOneOmni,
                                   bool  /*doRemoveBubbles*/, staticVector<point3d>* /*hexahsToExcludeFromResultingMesh*/,
                                   bool saveToWrl, point3d* hexah) // alphaQual=5.0f
{
    std::cout << "reconstructGC" << std::endl;
    long t_reconstructGC = clock();

    MaxFlow<int, float> maxFlowGraph(getNbCells());

    // fill s-t edges
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        assert(c.cellId > -1);
        float ws = c.cellSWeight;
        float wt = c.cellTWeight;

        assert(ws >= 0.0f);
        assert(wt >= 0.0f);
        assert(!std::isnan(ws));
        assert(!std::isnan(wt));

        maxFlowGraph.addNode(c.cellId, ws, wt);
    }

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

            assert(_cellsAttr[fu.cellIndex].cellId > -1);
            assert(_cellsAttr[fv.cellIndex].cellId > -1);

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

    long t_maxflow = clock();
    // Find graph-cut solution
    const float totalFlow = maxFlowGraph.compute();
    printfElapsedTime(t_maxflow, "Maxflow computation ");

    std::cout << "totalFlow: " << totalFlow << std::endl;

    // Update FULL/EMPTY status of all cells
    for(CellIndex i = 0; i < _cellsAttr.size(); ++i)
    {
        GC_cellInfo& c = _cellsAttr[i];
        c.full = !maxFlowGraph.isSource(i);
    }

    // Convert cells FULL/EMPTY into surface
    setIsOnSurface();

    if(saveToWrl)
    {
        std::string fileNameWrl = folderName + baseName + ".wrl";
        std::string fileNameWrlTex = folderName + baseName + "Textured.wrl";
        std::string fileNamePly = folderName + baseName + ".ply";
        saveMaxflowToWrl(folderName, fileNameTxt, fileNameTxtCam, fileNameWrl, fileNameWrlTex, fileNamePly,
                         camerasPerOneOmni, cams);
    }

    std::string resultFolderName = folderName + baseName + "/";
    bfs::create_directory(resultFolderName);

    freeUnwantedFullCells(resultFolderName, hexah);

    printfElapsedTime(t_reconstructGC, "ReconstructGC computation (full maxflow) ");

    std::cout << "reconstructGC end" << std::endl;
}

void mv_delaunay_GC::reconstructExpetiments(staticVector<int>* cams, std::string folderName,
                                            std::string fileNameStGraph, std::string fileNameStSolution,
                                            std::string fileNameTxt, std::string fileNameTxtCam, int camerasPerOneOmni,
                                            bool update, point3d* hexahInflated, std::string tmpCamsPtsFolderName,
                                            staticVector<point3d>* hexahsToExcludeFromResultingMesh, point3d spaceSteps)
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
                      fileNameTxtCam, camerasPerOneOmni, true, hexahsToExcludeFromResultingMesh, saveTemporaryWrlFiles, hexahInflated);

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
                      fileNameTxtCam, camerasPerOneOmni, true, hexahsToExcludeFromResultingMesh, saveTemporaryWrlFiles, hexahInflated);

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
                      fileNameTxtCam, camerasPerOneOmni, true, hexahsToExcludeFromResultingMesh, saveTemporaryWrlFiles, hexahInflated);

        if(saveTemporaryBinFiles)
            saveDhInfo(folderName + "delaunayTriangulationInfoAfterHallRemoving.bin");
    }
}

float mv_delaunay_GC::computeSurfaceArea()
{
    float area = 0.0f;
    // loop over all facets
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellsAttr[f1.cellIndex].full; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellsAttr[f2.cellIndex].full; // get if it is occupied

            if(uo != vo)
            {
                area += facetArea(f1);
            }
        }
    }

    return area;
}

mv_mesh* mv_delaunay_GC::createMesh()
{
    std::cout << "Extract mesh from GC" << std::endl;

    int nbSurfaceFacets = setIsOnSurface();

    mv_mesh* me = new mv_mesh();
    // TODO: copy only surface points and remap visibilities

    me->pts = new staticVector<point3d>(getNbVertices());
    for(const point3d& p: _verticesCoords)
    {
        me->pts->push_back(p);
    }

    me->tris = new staticVector<mv_mesh::triangle>(nbSurfaceFacets);
    // loop over all facets
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellsAttr[f1.cellIndex].full; // get if it is occupied
            if(!uo)
                continue;

            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellsAttr[f2.cellIndex].full; // get if it is occupied

            if(uo == vo)
                continue;

            VertexIndex vertices[3];
            vertices[0] = getVertexIndex(f1, 0);
            vertices[1] = getVertexIndex(f1, 1);
            vertices[2] = getVertexIndex(f1, 2);

            point3d points[3];
            for(int k = 0; k < 3; ++k)
            {
                points[k] = _verticesCoords[vertices[k]];
            }

            // if(T.is_infinite(f1.first->vertex(f1.second)))
            // {
            //    printf("WARNINIG infinite vertex\n");
            // }

            point3d D1 = _verticesCoords[getOppositeVertexIndex(f1)];
            point3d D2 = _verticesCoords[getOppositeVertexIndex(f2)];

            point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

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
                mv_mesh::triangle t;
                t.alive = true;
                t.i[0] = vertices[0];
                t.i[1] = vertices[1];
                t.i[2] = vertices[2];
                me->tris->push_back(t);
            }
            else
            {
                mv_mesh::triangle t;
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

staticVector<rgb>* mv_delaunay_GC::getPtsColorsByNCams()
{
    staticVector<rgb>* mePtsColors = new staticVector<rgb>(getNbVertices());

    for(VertexIndex i = 0; i < _verticesAttr.size(); ++i)
    {
        const GC_vertexInfo& v = _verticesAttr[i];

        rgb col;
        if(v.getNbCameras() < 3)
        {
            col.r = 0;
            col.g = 0;
            col.b = 0;
        }
        else
        {
            if(v.getNbCameras() == 3)
            {
                col.r = 0;
                col.g = 255;
                col.b = 0;
            }
            else
            {
                col.r = 0;
                col.g = 0;
                col.b = 255;
            }
        }
        mePtsColors->push_back(col);
    }

    return mePtsColors;
}

void mv_delaunay_GC::initTetrahedralizationFromMeshTrianglesCenter(mv_mesh* mesh, bool _addPointsToPreventSingularities)
{
    if(mp->verbose)
        printf("creating 3D delaunay triangulation from mesh triangles center\n");
    long t1 = initEstimate();

    point3d minP = mesh->computeTriangleCenterOfGravity(0);
    point3d maxP = minP;

    _verticesCoords.reserve(mesh->tris->size());
    _verticesAttr.reserve(mesh->tris->size());

    for(int i = 0; i < mesh->tris->size(); ++i)
    {
        point3d mep = mesh->computeTriangleCenterOfGravity(i);
        minP.x = std::min(mep.x, minP.x);
        minP.y = std::min(mep.y, minP.y);
        minP.z = std::min(mep.z, minP.z);
        maxP.x = std::max(mep.x, maxP.x);
        maxP.y = std::max(mep.y, maxP.y);
        maxP.z = std::max(mep.z, maxP.z);

        _verticesCoords.push_back(mep);

        GC_vertexInfo newv;
        newv.nrc = 0;
        newv.segSize = 0;
        newv.segId = -2;

        _verticesAttr.push_back(newv);

        printfEstimate(i, mesh->tris->size(), t1);
    } // for idTri
    finishEstimate();

    if(_addPointsToPreventSingularities)
    {
        std::array<point3d, 8> voxelCorners = {
            point3d(minP.x, minP.y, minP.z), // 0: orig
            point3d(maxP.x, minP.y, minP.z), // 1: X
            point3d(maxP.x, maxP.y, minP.z),
            point3d(minP.x, maxP.y, minP.z), // 3: Y
            point3d(minP.x, minP.y, maxP.z), // 4: Z
            point3d(maxP.x, minP.y, maxP.z),
            point3d(maxP.x, maxP.y, maxP.z),
            point3d(minP.x, maxP.y, maxP.z)
        };

        addPointsToPreventSingularities(&voxelCorners[0], 0.00001);
        addHelperPoints(10, &voxelCorners[0], 0.00001);
    }
    initVertices();
    computeDelaunay();
}

void mv_delaunay_GC::initTetrahedralizationFromMeshVertices(mv_mesh* mesh, bool _addPointsToPreventSingularities)
{
    if(mp->verbose)
        printf("creating 3D delaunay triangulation from mesh vertices\n");

    point3d minP = mesh->computeTriangleCenterOfGravity(0);
    point3d maxP = minP;

    _verticesCoords.reserve(mesh->pts->size());
    _verticesAttr.reserve(mesh->pts->size());

    for(int i = 0; i < mesh->pts->size(); ++i)
    {
        point3d mep = (*mesh->pts)[i];
        minP.x = std::min(mep.x, minP.x);
        minP.y = std::min(mep.y, minP.y);
        minP.z = std::min(mep.z, minP.z);
        maxP.x = std::max(mep.x, maxP.x);
        maxP.y = std::max(mep.y, maxP.y);
        maxP.z = std::max(mep.z, maxP.z);

        point3d p(mep.x, mep.y, mep.z);
        _verticesCoords.push_back(p);

        GC_vertexInfo newv;
        newv.nrc = 0;
        newv.segSize = 0;
        newv.segId = -2;
        _verticesAttr.push_back(newv);
    }

    if(_addPointsToPreventSingularities)
    {
        std::array<point3d, 8> voxelCorners = {
            point3d(minP.x, minP.y, minP.z), // 0: orig
            point3d(maxP.x, minP.y, minP.z), // 1: X
            point3d(maxP.x, maxP.y, minP.z),
            point3d(minP.x, maxP.y, minP.z), // 3: Y
            point3d(minP.x, minP.y, maxP.z), // 4: Z
            point3d(maxP.x, minP.y, maxP.z),
            point3d(maxP.x, maxP.y, maxP.z),
            point3d(minP.x, maxP.y, maxP.z)
        };

        addPointsToPreventSingularities(&voxelCorners[0], 0.00001);
        addHelperPoints(10, &voxelCorners[0], 0.00001);
    }
    initVertices();
    computeDelaunay();
}

staticVector<int>* mv_delaunay_GC::getNearestTrisFromMeshTris(mv_mesh* otherMesh)
{
    ///////////////////////////////////////////////////////////////////////////////////////
    if(mp->verbose)
        printf("getNearestTrisFromMeshTris\n");

    staticVector<int>* nearestTrisIds = new staticVector<int>();
    nearestTrisIds->resize(otherMesh->tris->size());

#pragma omp parallel for
    for(int otherMeshIdTri = 0; otherMeshIdTri < otherMesh->tris->size(); ++otherMeshIdTri)
    {
        point3d mep = otherMesh->computeTriangleCenterOfGravity(otherMeshIdTri);
        VertexIndex nvi = locateNearestVertex(mep);
        if(nvi != GEO::NO_VERTEX && _verticesAttr[nvi].segId == -2)
        {
            int idTriMe2 = nvi;
            (*nearestTrisIds)[otherMeshIdTri]= idTriMe2;
        }
        else
        {
            (*nearestTrisIds)[otherMeshIdTri] = -1;
        }
    }

    return nearestTrisIds;
}

staticVector<int>* mv_delaunay_GC::getNearestPtsFromMesh(mv_mesh& otherMesh)
{
    ///////////////////////////////////////////////////////////////////////////////////////
    if(mp->verbose)
        printf("getNearestPtsFromMesh\n");

    staticVector<int>* nearestPtsIds = new staticVector<int>();
    nearestPtsIds->resize(otherMesh.pts->size());

#pragma omp parallel for
    for(int otherMeshIdPt = 0; otherMeshIdPt < otherMesh.pts->size(); ++otherMeshIdPt)
    {
        point3d mep = (*otherMesh.pts)[otherMeshIdPt];
        VertexIndex nvi = locateNearestVertex(mep);
        if(nvi != GEO::NO_VERTEX && _verticesAttr[nvi].segId == -2)
        {
            (*nearestPtsIds)[otherMeshIdPt]= nvi;
        }
        else
        {
            (*nearestPtsIds)[otherMeshIdPt] = -1;
        }
    }

    if(mp->verbose)
        printf("getNearestPtsFromMesh end\n");

    return nearestPtsIds;
}

void mv_delaunay_GC::saveMeshColoredByCamsConsistency(const std::string& consistencyWrlFilepath, const std::string& nbCamsWrlFilepath)
{
    if(mp->verbose)
        std::cout << "saveMeshColoredByCamsConsistency: " << consistencyWrlFilepath << ", " << nbCamsWrlFilepath << std::endl;
    long timer = std::clock();
    int maxLevel = 2;

    mv_mesh* mesh = createMesh();
    staticVector<staticVector<int>*>* ptsCams = createPtsCams();

    // Export colors per cams consistency
    {
        staticVector<staticVector<int>*>* ptsNeighPts = mesh->getPtsNeighPtsOrdered();

        staticVector<float> ptsScore;
        ptsScore.resize(mesh->pts->size());
        //long t1 = initEstimate();
    #pragma omp parallel for
        for(int idPt = 0; idPt < mesh->pts->size(); idPt++)
        {
            staticVector<int> camsHist;
            camsHist.resize_with(mp->ncams, 0);
            int maxBinValue = 0;
            staticVector<int>* nptsids = mesh->getNearPointsIds(idPt, maxLevel, ptsNeighPts); // neighbors points
            nptsids->push_front(idPt); // add itself
            for(int i = 0; i < nptsids->size(); i++)
            {
                staticVector<int>* tcams = (*ptsCams)[(*nptsids)[i]];
                for(int c = 0; c < sizeOfStaticVector<int>(tcams); c++)
                {
                    int* h = &camsHist[(*tcams)[c]];
                    if(i == 0 || *h > 0)
                        (*h)++;
                    maxBinValue = std::max(maxBinValue, *h);
                }
            }
            ptsScore[idPt] = float(maxBinValue) / float(nptsids->size());
            delete nptsids;

            //printfEstimate(idPt, mesh->pts->size(), t1);
        }
        //finishEstimate();

        bool colorPerVertex = true;
        staticVector<rgb> colors;

        if(colorPerVertex)
        {
            colors.reserve(ptsScore.size());
            for(int idVertex = 0; idVertex < ptsScore.size(); ++idVertex)
            {
                colors.push_back(getColorFromJetColorMap(ptsScore[idVertex]));
            }
        }
        else
        {
            colors.reserve(mesh->tris->size());
            for(int idtri = 0; idtri < mesh->tris->size(); ++idtri)
            {
                // min score accross the vertices of the face
                float minScore = 1.0;
                for(int k = 0; k < 3; k++)
                {
                    minScore = std::min(minScore, ptsScore[(*mesh->tris)[idtri].i[k]]);
                }
                colors.push_back(getColorFromJetColorMap(minScore));
            }
        }

        mv_output3D o3d(mp);
        o3d.saveMvMeshToWrl(mesh, consistencyWrlFilepath, &colors, true, colorPerVertex);
        deleteArrayOfArrays<int>(&ptsNeighPts);
    }

    // Export colors per number of cameras
    {
        staticVector<rgb> colors;
        colors.reserve(ptsCams->size());
        int maxVisibility = 0;
        for(int idVertex = 0; idVertex < ptsCams->size(); ++idVertex)
        {
            maxVisibility = std::max(maxVisibility, (*ptsCams)[idVertex]->size());
        }
        for(int idVertex = 0; idVertex < ptsCams->size(); ++idVertex)
        {
            float score = float((*ptsCams)[idVertex]->size()) / float(maxVisibility);
            colors.push_back(getColorFromJetColorMap(score));
        }

        mv_output3D o3d(mp);
        o3d.saveMvMeshToWrl(mesh, nbCamsWrlFilepath, &colors, true, true);
    }

    deleteArrayOfArrays<int>(&ptsCams);
    delete mesh;

    printfElapsedTime(timer, "saveMeshColoredByCamsConsistency ");
    if(mp->verbose)
        std::cout << "saveMeshColoredByCamsConsistency end" << std::endl;
}

void mv_delaunay_GC::segmentFullOrFree(bool full, staticVector<int>** out_fullSegsColor, int& out_nsegments)
{
    if(mp->verbose)
        printf("segmenting connected space\n");

    staticVector<int>* colors = new staticVector<int>(_cellsAttr.size());
    colors->resize_with(_cellsAttr.size(), -1);

    staticVector<CellIndex>* buff = new staticVector<CellIndex>(_cellsAttr.size());

    int col = 0;

    // segment connected free space
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];

        if((!isInfiniteCell(ci)) && ((*colors)[c.cellId] == -1) && (c.full == full))
        {
            // backtrack all connected interior cells
            buff->resize(0);
            buff->push_back(ci);

            while(buff->size() > 0)
            {
                CellIndex tmp_ci = buff->pop();
                GC_cellInfo& tmp_c = _cellsAttr[tmp_ci];

                (*colors)[tmp_c.cellId] = col;

                for(int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization->cell_adjacent(tmp_ci, k);
                    if(nci == GEO::NO_CELL)
                        continue;
                    const GC_cellInfo& nc = _cellsAttr[nci];
                    if((!isInfiniteCell(nci)) && (nc.cellId > -1) && ((*colors)[nc.cellId] == -1) &&
                       (nc.full == full))
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

int mv_delaunay_GC::removeBubbles()
{
    int nbEmptySegments = 0;
    staticVector<int>* emptySegColors = nullptr;
    segmentFullOrFree(false, &emptySegColors, nbEmptySegments);

    if(mp->verbose)
        printf("removing bubbles\n");

    staticVectorBool* colorsToFill = new staticVectorBool(nbEmptySegments);
    // all free space segments which contains camera has to remain free all others full
    colorsToFill->resize_with(nbEmptySegments, true);

    // all free space segments which contains camera has to remain free
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& cell = _cellsAttr[ci];

        if(isInfiniteCell(ci) || (*emptySegColors)[cell.cellId] < 0)
            continue;

        const GC_vertexInfo& a = _verticesAttr[_tetrahedralization->cell_vertex(ci, 0)];
        const GC_vertexInfo& b = _verticesAttr[_tetrahedralization->cell_vertex(ci, 1)];
        const GC_vertexInfo& c = _verticesAttr[_tetrahedralization->cell_vertex(ci, 2)];
        const GC_vertexInfo& d = _verticesAttr[_tetrahedralization->cell_vertex(ci, 3)];
        if( a.isVirtual() || b.isVirtual() || c.isVirtual() || d.isVirtual())
        {
            // TODO FACA: check helper points are not connected to cameras?
            (*colorsToFill)[(*emptySegColors)[cell.cellId]] = false;
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

    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        if((!isInfiniteCell(ci)) && ((*emptySegColors)[c.cellId] >= 0) && ((*colorsToFill)[(*emptySegColors)[c.cellId]]))
        {
            c.full = true;
        }
    }

    delete colorsToFill;
    delete emptySegColors;

    if(mp->verbose)
        printf("nbubbles %i, all segs %i\n", nbubbles, nbEmptySegments);

    setIsOnSurface();

    return nbubbles;
}

int mv_delaunay_GC::removeDust(int minSegSize)
{
    if(mp->verbose)
        printf("removing dust\n");

    int nbFullSegments = 0;
    staticVector<int>* fullSegsColor = nullptr;
    segmentFullOrFree(true, &fullSegsColor, nbFullSegments);

    staticVector<int>* colorsSize = new staticVector<int>(nbFullSegments);
    colorsSize->resize_with(nbFullSegments, 0);

    // all free space segments which contains camera has to remain free
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        if((*fullSegsColor)[c.cellId] >= 0) // if we have a valid color: non empty and non infinit cell
        {
            (*colorsSize)[(*fullSegsColor)[c.cellId]] += 1; // count the number of cells in the segment
        }
    }

    int ndust = 0;
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        // if number of cells in the segment is too small, we change the status to "empty"
        if(((*fullSegsColor)[c.cellId] >= 0) && ((*colorsSize)[(*fullSegsColor)[c.cellId]] < minSegSize))
        {
            c.full = false;
            ++ndust;
        }
    }

    delete colorsSize;
    delete fullSegsColor;

    if(mp->verbose)
        printf("%i, all segs %i\n", ndust, nbFullSegments);

    setIsOnSurface();

    return ndust;
}

void mv_delaunay_GC::leaveLargestFullSegmentOnly()
{
    if(mp->verbose)
        printf("Largest full segment only.\n");

    int nsegments;
    staticVector<int>* colors = nullptr;
    segmentFullOrFree(true, &colors, nsegments);

    staticVector<int>* colorsSize = new staticVector<int>(nsegments);
    colorsSize->resize_with(nsegments, 0);

    // all free space segments which contains camera has to remain free
    int largestColor = -1;
    int maxn = 0;
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        int color = (*colors)[c.cellId];
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

    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        GC_cellInfo& c = _cellsAttr[ci];
        if((*colors)[c.cellId] != largestColor)
        {
            c.full = false;
        }
    }

    delete colorsSize;
    delete colors;

    setIsOnSurface();

    if(mp->verbose)
        printf("Largest full segment only. Done.\n");
}

staticVector<float>* mv_delaunay_GC::computeSegmentsSurfaceArea(bool full, staticVector<int>& colors, int nsegments)
{
    staticVector<float>* segmentsSurfAreas = new staticVector<float>(nsegments);
    segmentsSurfAreas->resize_with(nsegments, 0.0f);


    // loop over all facets
    for(CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        for(VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            Facet f2 = mirrorFacet(f1);
            if(isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool uo = _cellsAttr[f1.cellIndex].full; // get if it is occupied
            bool vo = _cellsAttr[f2.cellIndex].full; // get if it is occupied

            if(uo == vo)
                continue;

            if(!uo)
                continue;

            int col1 = colors[_cellsAttr[f1.cellIndex].cellId];
            int col2 = colors[_cellsAttr[f2.cellIndex].cellId];

            if((uo == full) && (col1 > -1))
            {
                (*segmentsSurfAreas)[col1] += facetArea(f1);
            }
            if((vo == full) && (col2 > -1))
            {
                (*segmentsSurfAreas)[col2] += facetArea(f1);
            }
        }
    }

    return segmentsSurfAreas;
}

staticVector<staticVector<int>*>* mv_delaunay_GC::createPtsCamsForAnotherMesh(staticVector<staticVector<int>*>* refPtsCams, mv_mesh& otherMesh)
{
    std::cout << "createPtsCamsForAnotherMesh" << std::endl;

    staticVector<int>* otherObjPtsToNearestPts = getNearestPtsFromMesh(otherMesh);

    staticVector<staticVector<int>*>* otherMeshPtsCams = new staticVector<staticVector<int>*>();
    otherMeshPtsCams->resize(otherObjPtsToNearestPts->size());

    for(int i = 0; i < otherObjPtsToNearestPts->size(); ++i)
    {
        staticVector<int>* pOther = new staticVector<int>();
        (*otherMeshPtsCams)[i] = pOther; // give ownership
        int iRef = (*otherObjPtsToNearestPts)[i];
        if(iRef == -1)
            continue;
        staticVector<int>* pRef = (*refPtsCams)[iRef];
        if(pRef == nullptr)
            continue;

        *pOther = *pRef;
    }

    std::cout << "createPtsCamsForAnotherMesh end" << std::endl;
    return otherMeshPtsCams;
}
