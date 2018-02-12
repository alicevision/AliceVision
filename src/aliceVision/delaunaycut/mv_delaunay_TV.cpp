// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_delaunay_TV.hpp"

#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/delaunaycut/hallucinations.hpp>


void mv_delanuay_TV::updateHistogram(GC_Cell_handle ch, int rc, float depths, float voxSize, float sigma, int weight)
{
    int id = ch->info().cellId;
    point3d p = cellCentreOfGravity(ch);
    float depthp = (mp->CArr[rc] - p).size();

    int nsigma = (int)(fabs(depthp - depths) / voxSize);
    if(depthp < depths)
    {
        nsigma = -nsigma;
    }

    if(abs(nsigma) <= sigma)
    {
        float aa = (float)nsigma / (float)sigma; // in <-1,1>
        aa = ((aa + 1.0f) / 2.0f) * 7.0f;        // in <0,7> R
        int binid = (int)(aa + 0.5f);            // round ... in <0,7> N
        // binid = min(8,binid);
        // binid = max(0,binid);
        (*tvData)[id].hist[binid] += weight;
    }
    else
    {
        if(depthp > depths)
        {
            // is behind surface (ocluded voxel)
            int binid = 8;
            (*tvData)[id].hist[binid] += weight;
        }
        else
        {
            // is in front of surface (empty voxel)
            int binid = 9;
            (*tvData)[id].hist[binid] += weight;
        }
    }
}

void mv_delanuay_TV::initTvData(staticVector<int>* cams, float voxSize, float sigma)
{
    int w = mp->mip->imp.width;
    int h = mp->mip->imp.height;

    int nV = 0; // number of tetrahedrons
    for(GC_Dh::Finite_cells_iterator fit = T.finite_cells_begin(); fit != T.finite_cells_end(); ++fit)
    {
        assert(fit->info().cellId > -1);
        if(fit->info().cellId != nV)
        {
            printf("WARNING different cellId  %i vs. %i!!!\n", fit->info().cellId, nV);
        }
        nV++;
    }

    tvData = new staticVector<TV_cellInfo>(nV);

    // init histograms
    for(GC_Dh::Finite_cells_iterator fit = T.finite_cells_begin(); fit != T.finite_cells_end(); ++fit)
    {
        TV_cellInfo tvc;
        for(int j = 0; j < 10; j++)
        {
            tvc.hist[j] = 0;
        }
        tvc.n = point3d(0.0f, 0.0f, 0.0f);
        tvc.p = point3d(0.0f, 0.0f, 0.0f);
        tvc.u = 0.0f;
        tvc.u_ = 0.0f;
        tvData->push_back(tvc);
    }

    /*
    //fill histograms
    printf("computing histograms\n");
    long t1=initEstimate();
    for (int c=0;c<cams->size();c++) {
            int rc = (*cams)[c];
            staticVector<float> *depthMap = loadArrayFromFile<float>(mv_getFileName(mp->mip, rc+1,
    mp->mip->MV_FILE_TYPE_depthMap, 1));
            staticVector<float> *simMap = loadArrayFromFile<float>(mv_getFileName(mp->mip, rc+1,
    mp->mip->MV_FILE_TYPE_simMap, 1));

            for (GC_Dh::Finite_cells_iterator fit=T.finite_cells_begin();fit!=T.finite_cells_end();++fit) {
                    int id = fit->info().cellId;
                    GC_Cell_handle ch = fit;
                    point3d p = cellCentreOfGravity(ch);
                    pixel pix; mp->getPixelFor3DPoint(&pix, p, rc);
                    if (mp->isPixelInImage(pix,1)==true) {
                            int pixi = pix.x*h+pix.y;
                            float depths = (*depthMap)[pixi];
                            if (depths > 0.0f) {
                                    updateHistogram(ch, rc, depths, voxSize, sigma,1);
                            };
                    };
            };

            delete depthMap;
            delete simMap;
            printfEstimate(c, cams->size(), t1);
    };//for i
    finishEstimate();
    */

    // fill histograms
    printf("computing histograms\n");
    long t2 = initEstimate();
    int id = 0;
    int numvert = (int)T.number_of_vertices();
    for(GC_Dh::Finite_vertices_iterator fit = T.finite_vertices_begin(); fit != T.finite_vertices_end(); ++fit)
    {
        if(fit->info().cams != NULL)
        {
            staticVector<int>* cams = fit->info().cams;
            for(int c = 0; c < cams->size(); c++)
            {
                int cam = (*cams)[c];
                GC_Cell_handle cv = getFacetInFrontVertexOnTheRayToTheCam((GC_Vertex_handle)fit, cam).first;
                // point3d p = convertPointToPoint3d(fit->point());
                point3d po = fit->info().point;
                point3d p = fit->info().point;
                float depths = (mp->CArr[cam] - p).size();
                int weight = fit->info().nrc;

                GC_Cell_handle lastFinite = NULL;
                point3d lastLpi;
                bool ok = (cv != NULL);
                while(ok)
                {
                    GC_Facet f1, f2;
                    point3d lpi;
                    // find cell which is nearest to the cam and which is intersected with cam-p ray
                    if(nearestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, cv, f1, f2, lpi) == false)
                    {
                        ok = false;
                    }
                    else
                    {
                        updateHistogram(f2.first, cam, depths, voxSize, sigma, weight);
                        cv = f2.first;
                        lastFinite = f2.first;
                        lastLpi = lpi;
                    }
                }

                int nbehind = 0;
                p = po; // HAS TO BE HERE !!!
                cv = getFacetBehindVertexOnTheRayToTheCam((GC_Vertex_handle)fit, cam).first;
                lastFinite = NULL;
                ok = (cv != NULL);
                while(ok)
                {
                    GC_Facet f1, f2;
                    point3d lpi;
                    // find cell which is farest to the cam and which is intersected with cam-p ray
                    if(farestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, cv, f1, f2, lpi) == false)
                    {
                        ok = false;
                    }
                    else
                    {
                        updateHistogram(f1.first, cam, depths, voxSize, sigma, weight);
                        cv = f2.first;
                        lastFinite = f2.first;
                        nbehind++;
                    }
                }

            } // for c
        }     // if (fit->info().cams != NULL)

        printfEstimate(id, numvert, t2);
        id++;
    } // for i
    finishEstimate();

    staticVector<point3d>* pts = new staticVector<point3d>(nV);
    staticVector<voxel>* cls = new staticVector<voxel>(nV);

    // init energy data
    for(GC_Dh::Finite_cells_iterator fit = T.finite_cells_begin(); fit != T.finite_cells_end(); ++fit)
    {
        int id = fit->info().cellId;
        int b = (int)(*tvData)[id].hist[3] + (int)(*tvData)[id].hist[4];
        float val = 1.0f;
        if(b >= 2)
        {
            val = -1.0f;
        }
        (*tvData)[id].u = val;
        (*tvData)[id].u_ = val;
        (*tvData)[id].n = point3d(0.0f, 0.0f, 0.0f);
        (*tvData)[id].p = point3d(0.0f, 0.0f, 0.0f);

        /*
        if (((*tvData)[id].hist[8]-(*tvData)[id].hist[9])>3)
        {
                point3d p = cellCentreOfGravity(fit);
                pts->push_back(p);
                voxel c = voxel(0,0,0);
                c.x = (int)((*tvData)[id].hist[8]>3)*255;
                c.y = (int)(((*tvData)[id].hist[8]-(*tvData)[id].hist[9])>3)*255;
                c.z = (int)((*tvData)[id].hist[9]>3)*255;
                cls->push_back(c);
        };
        */

        point3d p = cellCentreOfGravity(fit);
        pts->push_back(p);
        voxel c = voxel(0, 0, 0);
        if((*tvData)[id].hist[8] > (*tvData)[id].hist[9])
        {
            c.x = 255;
        }
        else
        {
            c.z = 255;
        }

        int sumMid = 0;
        for(int i = 0; i < 8; i++)
        {
            sumMid += (*tvData)[id].hist[i];
        }

        if(sumMid > ((*tvData)[id].hist[8] + (*tvData)[id].hist[9]))
        {
            c.y = 255;
        }

        cls->push_back(c);
    }

    delete pts;
    delete cls;
}

void mv_delanuay_TV::saveStatistic(float voxSize, float sigma)
{
    int RC = 4227;

    FILE* fStat = fopen("stat.txt", "w");
    fprintf(fStat, "%f %f\n", voxSize, sigma);
    fclose(fStat);

    FILE* fStats = fopen("statStats.txt", "w");
    FILE* fPts = fopen("statPts.txt", "w");
    FILE* fHists = fopen("statHists.txt", "w");
    FILE* fTetrsPts = fopen("statTetrsPts.txt", "w");

    // fill histograms
    printf("computing histograms\n");
    long t2 = initEstimate();
    int id = 0;
    int numvert = (int)T.number_of_vertices();
    for(GC_Dh::Finite_vertices_iterator fit = T.finite_vertices_begin(); fit != T.finite_vertices_end(); ++fit)
    {
        pixel RCPIX;
        mp->getPixelFor3DPoint(&RCPIX, fit->info().point, RC);
        if((fit->info().cams != NULL) && (mp->isPixelInImage(RCPIX) == true))
        {
            int cam = RC;
            GC_Cell_handle cv = getFacetInFrontVertexOnTheRayToTheCam((GC_Vertex_handle)fit, cam).first;
            // point3d p = convertPointToPoint3d(fit->point());
            point3d po = fit->info().point;
            point3d p = fit->info().point;
            float depths = (mp->CArr[cam] - p).size();
            int weight = fit->info().nrc;

            GC_Cell_handle lastFinite = NULL;
            point3d lastLpi;
            bool ok = (cv != NULL);
            while(ok)
            {
                GC_Facet f1, f2;
                point3d lpi;
                // find cell which is nearest to the cam and which is intersected with cam-p ray
                if(nearestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, cv, f1, f2, lpi) == false)
                {
                    ok = false;
                }
                else
                {
                    int cellId = cv->info().cellId;
                    fprintf(fStats, "%i %i %i %i\n", RCPIX.x, RCPIX.y, 0, fit->info().id);
                    fprintf(fPts, "%f %f %f\n", p.x, p.y, p.z);
                    fprintf(fHists, "%i %i %i %i %i %i %i %i %i %i\n", (*tvData)[cellId].hist[8],
                            (*tvData)[cellId].hist[0], (*tvData)[cellId].hist[1], (*tvData)[cellId].hist[2],
                            (*tvData)[cellId].hist[3], (*tvData)[cellId].hist[4], (*tvData)[cellId].hist[5],
                            (*tvData)[cellId].hist[6], (*tvData)[cellId].hist[7], (*tvData)[cellId].hist[9]);

                    fprintf(fTetrsPts, "%f %f %f %f %f %f %f %f %f %f %f %f\n", cv->vertex(0)->info().point.x,
                            cv->vertex(0)->info().point.y, cv->vertex(0)->info().point.z, cv->vertex(1)->info().point.x,
                            cv->vertex(1)->info().point.y, cv->vertex(1)->info().point.z, cv->vertex(2)->info().point.x,
                            cv->vertex(2)->info().point.y, cv->vertex(2)->info().point.z, cv->vertex(3)->info().point.x,
                            cv->vertex(3)->info().point.y, cv->vertex(3)->info().point.z);

                    cv = f2.first;
                    lastFinite = f2.first;
                    lastLpi = lpi;
                }
            }

            int nbehind = 0;
            p = po; // HAS TO BE HERE !!!
            cv = getFacetBehindVertexOnTheRayToTheCam((GC_Vertex_handle)fit, cam).first;
            lastFinite = NULL;
            ok = (cv != NULL);
            while(ok)
            {
                GC_Facet f1, f2;
                point3d lpi;
                // find cell which is farest to the cam and which is intersected with cam-p ray
                if(farestNeighCellToTheCamOnTheRay(mp->CArr[cam], p, cv, f1, f2, lpi) == false)
                {
                    ok = false;
                }
                else
                {
                    int cellId = cv->info().cellId;
                    fprintf(fStats, "%i %i %i %i\n", RCPIX.x, RCPIX.y, 1, fit->info().id);
                    fprintf(fPts, "%f %f %f\n", p.x, p.y, p.z);
                    fprintf(fHists, "%i %i %i %i %i %i %i %i %i %i\n", (*tvData)[cellId].hist[8],
                            (*tvData)[cellId].hist[0], (*tvData)[cellId].hist[1], (*tvData)[cellId].hist[2],
                            (*tvData)[cellId].hist[3], (*tvData)[cellId].hist[4], (*tvData)[cellId].hist[5],
                            (*tvData)[cellId].hist[6], (*tvData)[cellId].hist[7], (*tvData)[cellId].hist[9]);

                    fprintf(fTetrsPts, "%f %f %f %f %f %f %f %f %f %f %f %f\n", cv->vertex(0)->info().point.x,
                            cv->vertex(0)->info().point.y, cv->vertex(0)->info().point.z, cv->vertex(1)->info().point.x,
                            cv->vertex(1)->info().point.y, cv->vertex(1)->info().point.z, cv->vertex(2)->info().point.x,
                            cv->vertex(2)->info().point.y, cv->vertex(2)->info().point.z, cv->vertex(3)->info().point.x,
                            cv->vertex(3)->info().point.y, cv->vertex(3)->info().point.z);

                    cv = f2.first;
                    lastFinite = f2.first;
                    nbehind++;
                }
            }

        } // if (fit->info().cams != NULL)

        printfEstimate(id, numvert, t2);
        id++;
    } // for i
    finishEstimate();

    fclose(fStats);
    fclose(fPts);
    fclose(fHists);
    fclose(fTetrsPts);
}

/*
float mv_delanuay_TV::getInterpolatedU(point3d tp)
{
        GC_Point sp = GC_Point(tp.x,tp.y,tp.z);
        GC_Cell_handle c = T.locate(sp);
        point3d p0 = c->vertex(0)->info().point;
        point3d p1 = c->vertex(1)->info().point;
        point3d p2 = c->vertex(2)->info().point;
        point3d p3 = c->vertex(3)->info().point;

        float u0 = (*tvData)[c->vertex(0)->info().cellId].u;
        float u1 = (*tvData)[c->vertex(1)->info().cellId].u;
        float u2 = (*tvData)[c->vertex(2)->info().cellId].u;
        float u3 = (*tvData)[c->vertex(3)->info().cellId].u;


        //interpolation on tetrahedron

        pn = cross((p2-p1).normalize(),(p3-p1).normalize());
        v = (tp-p0).normalize();
        point3d vp123 = linePlaneIntersect(p0,v,p1,pn);



}
*/

void mv_delanuay_TV::compute_primal_energy(staticVector<int>* cams, float voxSize, float sigma)
{

    for(GC_Dh::Finite_cells_iterator fit = T.finite_cells_begin(); fit != T.finite_cells_end(); ++fit)
    {
        int id = fit->info().cellId;
        point3d p = cellCentreOfGravity(fit);
        point3d px = p + vx * sx;
        point3d py = p + vy * sy;
        point3d pz = p + vz * sz;

        /*

        u_x = u_shared[tz][ty][tx+1] - u_shared[tz][ty][tx];
        u_y = u_shared[tz][ty+1][tx] - u_shared[tz][ty][tx];
        u_z = u_shared[tz+1][ty][tx] - u_shared[tz][ty][tx];

*/
    }
}

void mv_delanuay_TV::runMaxflowPrepareToFileTV(std::string fileNameStGraph, float CONSTalphaVIS, float CONSTalphaPHOTO,
                                               float CONSTalphaAREA, float CONSTS, float CONSTT)
{
    printf("preparing s-t graph to file\n");

    float maxThr = 100.0f;

    FILE* f = fopen(fileNameStGraph.c_str(), "wb");

    int nV = 0; // number of tetrahedrons
    for(GC_Dh::Finite_cells_iterator fit = T.finite_cells_begin(); fit != T.finite_cells_end(); ++fit)
    {
        assert(fit->info().cellId > -1);
        if(fit->info().cellId != nV)
        {
            printf("WARNING different cellId  %i vs. %i!!!\n", fit->info().cellId, nV);
        }
        nV++;
    }

    int nE = 0;
    for(GC_Dh::Finite_facets_iterator it = T.finite_facets_begin(); it != T.finite_facets_end(); ++it)
    {
        GC_Facet fu = *it;
        GC_Facet fv = T.mirror_facet(fu);
        if((fu.first->info().cellId > -1) && (fv.first->info().cellId > -1) && (fu.first->info().cellId < nV) &&
           (fv.first->info().cellId < nV))
        {
            nE++;
        }

        if(fu.first->info().cellId >= nV)
        {
            printf("WARNING large cellId  %i max %i!!!\n", fu.first->info().cellId, nV);
        }

        if(fv.first->info().cellId >= nV)
        {
            printf("WARNING large cellId  %i max %i!!!\n", fv.first->info().cellId, nV);
        }
    }

    fwrite(&nV, sizeof(int), 1, f);
    fwrite(&nE, sizeof(int), 1, f);

    // fill s-t edges
    for(GC_Dh::Finite_cells_iterator fit = T.finite_cells_begin(); fit != T.finite_cells_end(); ++fit)
    {
        assert(fit->info().cellId > -1);

        /*
        float ws = CONSTS*fit->info().cellSWeight;
        float wt = CONSTT*fit->info().cellTWeight;
        */
        int id = fit->info().cellId;
        float ws = 0.0f;
        float wt = 0.0f;

        /*
        if ((*tvData)[id].hist[8] > (*tvData)[id].hist[9]) {
                wt = fabs((float)((*tvData)[id].hist[8]-(*tvData)[id].hist[9]));
        }else{
                ws = fabs((float)((*tvData)[id].hist[8]-(*tvData)[id].hist[9]));
        };
        */

        /*
        ws = (float)(*tvData)[id].hist[9]/(float)((*tvData)[id].hist[8]+1);
        wt = (float)(*tvData)[id].hist[8]/(float)((*tvData)[id].hist[9]+1);
        */

        // histGrid210-60
        if(((float)((*tvData)[id].hist[8] + 1) / (float)((*tvData)[id].hist[9] + 1) > 10.0f) &&
           ((*tvData)[id].hist[8] > 1000))
        {
            wt = (float)((*tvData)[id].hist[8] + 1);
        }
        ws = (float)((*tvData)[id].hist[9] + 1);

        if((ws < 0.0f) || (wt < 0.0f) || (std::isnan(ws) == true) || (std::isnan(wt) == true))
        {
            printf("WARNING ws or wt is < 0 !!! %f %f %f %f \n", CONSTS, fit->info().cellSWeight, CONSTT,
                   fit->info().cellTWeight);
        }

        fwrite(&fit->info().cellId, sizeof(int), 1, f);
        fwrite(&ws, sizeof(float), 1, f);
        fwrite(&wt, sizeof(float), 1, f);
    }

    // fill u-v directed edges
    for(GC_Dh::Finite_facets_iterator it = T.finite_facets_begin(); it != T.finite_facets_end(); ++it)
    {
        GC_Facet fu = *it;
        GC_Facet fv = T.mirror_facet(fu);
        if((fu.first->info().cellId > -1) && (fv.first->info().cellId > -1) && (fu.first->info().cellId < nV) &&
           (fv.first->info().cellId < nV))
        {
            /*
            float w1 = fu.first->info().gEdgeVisWeight[fu.second]*CONSTalphaVIS +
                               fu.first->info().gEdgePhotoWeight[fu.second]*CONSTalphaPHOTO;

            float w2 = fv.first->info().gEdgeVisWeight[fv.second]*CONSTalphaVIS +
                               fv.first->info().gEdgePhotoWeight[fv.second]*CONSTalphaPHOTO;
            */
            float w1 = 1.0f;
            float w2 = 1.0f;

            if((w1 < 0.0f) || (w2 < 0.0f) || (std::isnan(w1) == true) || (std::isnan(w2) == true))
            {
                printf("WARNING w1 or w2 is < 0 !!! f1 vis %f %f, f1 photo %f %f, f2 vis %f %f, f2 photo %f %f \n",
                       CONSTalphaVIS, fu.first->info().gEdgeVisWeight[fu.second],
                       // CONSTalphaPHOTO, fu.first->info().gEdgePhotoWeight[fu.second],
                       CONSTalphaVIS, fv.first->info().gEdgeVisWeight[fv.second] //,
                       // CONSTalphaPHOTO, fv.first->info().gEdgePhotoWeight[fv.second]
                       );
            }

            fwrite(&fu.first->info().cellId, sizeof(int), 1, f);
            fwrite(&fv.first->info().cellId, sizeof(int), 1, f);
            fwrite(&w1, sizeof(float), 1, f);
            fwrite(&w2, sizeof(float), 1, f);
        }
    }

    fclose(f);
}

bool mv_delanuay_TV::reconstructVoxelTV(point3d voxel[8], staticVector<int>* voxelsIds, std::string folderName,
                                        float inflateHexahfactor, std::string tmpCamsPtsFolderName, int numSubVoxs)
{
    point3d hexahInflated[8];

    printf("hexahedron inflate factor is %f\n", inflateHexahfactor);

    if(inflateHexahfactor == 1.0f)
    {
        for(int i = 0; i < 8; i++)
        {
            hexahInflated[i] = voxel[i];
        }
    }
    else
    {
        inflateHexahedron(voxel, hexahInflated, inflateHexahfactor);
    }

    staticVector<int>* cams = pc->findCamsWhichInteresctsHexahedron(hexahInflated);

    if(cams->size() < 1)
    {
        delete cams;
        return false;
    }

    int camerasPerOneOmni = 1;
    std::string fileNameDh = folderName + "delanuayTrinagluation.bin";
    std::string fileNameInfo = folderName + "delanuayTrinagluationInfo.bin";
    std::string fileNameStGraph = folderName + "stGraph.bin";
    std::string fileNameStSolution = folderName + "stGraphSolution.bin";
    std::string fileNameTxt = folderName + "delanuayTrinaglesMaxflow.txt";
    std::string fileNameTxtCam = folderName + "delanuayTrinaglesCamerasForColoringMaxflow.txt";
    std::string fileNameDelanuayVerticesWrl = folderName + "delanuayVertices.wrl";
    std::string fileNameDelanuayVerticesSegWrl = folderName + "delanuayVerticesSeg.wrl";
    std::string fileNameCams = folderName + "cams.bin";
    std::string fileNameDelanuayVerticesSegFilteredWrl = folderName + "delanuayVerticesSegFiltered.wrl";

    saveArrayToFile<int>(fileNameCams, cams);

    // createTriangulationFromDepthMapsCamsVoxel(cams,fileNameDelanuayVerticesWrl, 0, voxelsIds, hexahInflated);
    createTriangulationFromDepthMapsCamsVoxelGrid(cams, fileNameDelanuayVerticesWrl, 0, voxelsIds, hexahInflated,
                                                  numSubVoxs, true);

    float voxSize = point3d(sx, sy, sz).size();
    float sigma = 5.0f * voxSize;
    initTvData(cams, voxSize, sigma);

    saveStatistic(voxSize, sigma);

    // computeVerticesSegSize(fileNameDelanuayVerticesSegWrl);
    // removeSmallSegs(fileNameDelanuayVerticesSegFilteredWrl);

    /*
            fillGraph(
                    (bool)mp->mip->_ini.get<bool>("delanuaycut.negVisIn", true),
                    (bool)mp->mip->_ini.get<bool>("delanuaycut.negVisOut", true),
                    (bool)mp->mip->_ini.get<bool>("delanuaycut.facetConfPhoto", true),
                    (float)mp->mip->_ini.get<double>("delanuaycut.sigmaPixelSize", 0.0)
            );


            updateGraphFromTmpPtsCamsHexah((bool)mp->mip->_ini.get<bool>("delanuaycut.negVisIn", true),
                    (bool)mp->mip->_ini.get<bool>("delanuaycut.negVisOut", true),
                    (bool)mp->mip->_ini.get<bool>("delanuaycut.facetConfPhoto", true),
                    (float)mp->mip->_ini.get<double>("delanuaycut.sigmaPixelSize", 0.0),
                    cams, hexahInflated, tmpCamsPtsFolderName);

    */

    float CONSTalphaVIS = (float)mp->mip->_ini.get<double>("delanuaycut.CONSTalphaVISI", 10000.0);
    float CONSTalphaPHOTO = (float)mp->mip->_ini.get<double>("delanuaycut.CONSTalphaPHOT", 1000.0);
    float CONSTalphaAREA = (float)mp->mip->_ini.get<double>("delanuaycut.CONSTalphaAREA", 100000.0);
    float CONSTS = (float)mp->mip->_ini.get<double>("delanuaycut.CONSTS", 1.0);
    float CONSTT = (float)mp->mip->_ini.get<double>("delanuaycut.CONSTT", 10000.0);
    runMaxflowPrepareToFileTV(fileNameStGraph, CONSTalphaVIS, CONSTalphaPHOTO, CONSTalphaAREA, CONSTS, CONSTT);
    mv_delanuay_GC::tgraph* g = runMaxflowFillGraphFromFile(fileNameStGraph);
    runMaxflow(g, fileNameStSolution);
    delete g;
    runMaxflowReadSolution(fileNameStSolution);

    std::string fileNameWrl = folderName + "delanuayTrinaglesMaxflow.wrl";
    std::string fileNameWrlTex = folderName + "delanuayTrinaglesMaxflowTextured.wrl";
    std::string fileNamePly = folderName + "delanuayTrinaglesMaxflow.ply";
    saveMaxflowToWrl(folderName, fileNameTxt, fileNameTxtCam, fileNameWrl, fileNameWrlTex, fileNamePly,
                     camerasPerOneOmni, cams);

    saveDhAndDeallocate(fileNameDh, fileNameInfo);

    smooth(mp, fileNameTxt, "delanuayTrinaglesSmoothTextured.wrl", "delanuayTrinaglesSmoothTextured.ply", folderName, 1,
           true);
    // filterLargeTrianglesMeshDist(mp, pc, folderName, "meshTrisAreaColored.wrl", "meshAreaConsistentTextured.wrl",
    // "meshAreaConsistent.ply", "meshAreaConsistent.wrl");

    delete cams;

    return true;
}
