// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_charts_packer.hpp"

#include <aliceVision/output3D/mv_output3D.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>


int qSortCompareChartAsc(const void* ia, const void* ib)
{
    mv_charts_packer::chart* a = *(mv_charts_packer::chart**)ia;
    mv_charts_packer::chart* b = *(mv_charts_packer::chart**)ib;
    if(a->npixels > b->npixels)
    {
        return 1;
    }

    return -1;
}

mv_charts_packer::chart::chart(int _segid, int _ntris, mv_mesh_unwrap* mew)
{
    segid = _segid;
    ntris = _ntris;
    atlasid = -1;
    npixels = 0;

    // printf("segid %i segs %i ntris %i",segid,mew->univ->allelems,ntris);

    textureTiranglesMeshIds = new staticVector<int>(ntris);

    rc = -1;
    int ncams = mew->univ->elts[segid].cams->nbits;
    for(int c = 0; c < ncams; c++)
    {
        if(mew->univ->elts[segid].cams->getbit(c))
        {
            rc = (*mew->usedcams)[c];
        }
    }

    // printf("rc %i ncams %i nbits %i\n",rc,mew->usedcams->size(),mew->univ->elts[segid].cams->nbits);

    LU.x = std::numeric_limits<int>::max();
    LU.y = std::numeric_limits<int>::max();
    RD.x = std::numeric_limits<int>::min();
    RD.y = std::numeric_limits<int>::min();

    uhorizon = nullptr;
    dhorizon = nullptr;
}

mv_charts_packer::chart::~chart()
{
    delete textureTiranglesMeshIds;
    if(uhorizon != nullptr)
    {
        delete uhorizon;
    }
    if(dhorizon != nullptr)
    {
        delete dhorizon;
    }
}

void mv_charts_packer::chart::addTriangle(int triid, mv_mesh_unwrap* mew)
{
    textureTiranglesMeshIds->push_back(triid);
    mv_mesh::triangle_proj tp =
        mew->getTriangleProjection(triid, mew->mp, rc, mew->mp->mip->getWidth(rc), mew->mp->mip->getHeight(rc));
    LU.x = std::min(LU.x, tp.lu.x);
    LU.y = std::min(LU.y, tp.lu.y);
    RD.x = std::max(RD.x, tp.rd.x);
    RD.y = std::max(RD.y, tp.rd.y);
    // printf("rc %i tris %i triid %i allocated %i, size
    // %i\n",rc,mew->tris->size(),triid,textureTiranglesMeshIds->reserved(),textureTiranglesMeshIds->size());
    // printf("%i %i %i %i\n",LU.x, LU.y, RD.x, RD.y);
}

void mv_charts_packer::chart::addTriangleToHorizons(int triid, mv_mesh_unwrap* mew)
{
    mv_mesh::triangle_proj tp =
        mew->getTriangleProjection(triid, mew->mp, rc, mew->mp->mip->getWidth(rc), mew->mp->mip->getHeight(rc));

    //if(mew->mp->verbose)
    //    printf("addTriangleToHorizons triid %i, rc %i, bb %i %i %i %i\n", triid, rc, tp.lu.x, tp.lu.y, tp.rd.x,
    //           tp.rd.y);

    pixel pix;
    for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
    {
        for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
        {
            mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
            if(mew->doesTriangleIntersectsRectangle(&tp, &re))
            {
                (*uhorizon)[pix.x - LU.x] = std::min((*uhorizon)[pix.x - LU.x], pix.y - LU.y);
                (*dhorizon)[pix.x - LU.x] = std::max((*dhorizon)[pix.x - LU.x], pix.y - LU.y);
            }
        } // for y
    }     // for x
}

void mv_charts_packer::chart::createHorizons(int gutter, mv_mesh_unwrap* mew)
{
    int hw = RD.x - LU.x + 1 + 2 * gutter;
    // printf("%i %i %i %i %i\n",LU.x, LU.y, RD.x, RD.y, hw);

    uhorizon = new staticVector<int>(hw);
    uhorizon->resize_with(hw, RD.y - LU.y);
    dhorizon = new staticVector<int>(hw);
    dhorizon->resize_with(hw, 0);

    for(int i = 0; i < textureTiranglesMeshIds->size(); i++)
    {
        // if (mew->mp->verbose) printf("addTriangleToHorizons %i of %i\n",i,textureTiranglesMeshIds->size());
        addTriangleToHorizons((*textureTiranglesMeshIds)[i], mew);
        // if (mew->mp->verbose) printf("done\n");
    }

    LU.x -= gutter;
    LU.y -= gutter;
    RD.x += gutter;
    RD.y += gutter;

    for(int i = hw - 1 - gutter; i >= gutter; i--)
    {
        (*uhorizon)[i] = (*uhorizon)[i - gutter];
        (*dhorizon)[i] = (*dhorizon)[i - gutter];
    }

    for(int i = 0; i < gutter; i++)
    {
        (*uhorizon)[i] = (*uhorizon)[gutter];
        (*dhorizon)[i] = (*dhorizon)[gutter];
        (*uhorizon)[hw - 1 - i] = (*uhorizon)[hw - 1 - gutter];
        (*dhorizon)[hw - 1 - i] = (*dhorizon)[hw - 1 - gutter];
    }

    for(int i = 0; i < hw; i++)
    {
        (*dhorizon)[i] += 2 * gutter;
    }

    npixels = 0;
    for(int i = 0; i < hw; i++)
    {
        npixels += (*dhorizon)[i] - (*uhorizon)[i] + 1;
    }

    // showChart();
}

void mv_charts_packer::chart::showChart()
{
    int wmap = RD.x - LU.x + 1;
    int hmap = RD.y - LU.y + 1;
    float* data = new float[wmap * hmap];
    for(int i = 0; i < wmap * hmap; i++)
    {
        data[i] = 0.0f;
    }
    for(int i = 0; i < wmap; i++)
    {
        data[i * hmap + (*uhorizon)[i]] = 0.5f;
        data[i * hmap + (*dhorizon)[i]] = 1.0f;
    }
    showImageOpenCV(data, wmap, hmap, 0.0f, 1.0f);
    delete[] data;
}

bool mv_charts_packer::chart::needsToDivide(int textureSide)
{
    return ((RD.x - LU.x + 1 > textureSide) || (RD.y - LU.y + 1 > textureSide));
}

void mv_charts_packer::chart::divide(mv_mesh_unwrap* mew, int textureSide, mv_charts_packer::chart** _out1,
                                     mv_charts_packer::chart** _out2)
{
    staticVector<int>* textureTiranglesMeshIds1 = new staticVector<int>(textureTiranglesMeshIds->size());
    staticVector<int>* textureTiranglesMeshIds2 = new staticVector<int>(textureTiranglesMeshIds->size());
    for(int i = 0; i < textureTiranglesMeshIds->size(); i++)
    {
        int triid = (*textureTiranglesMeshIds)[i];
        mv_mesh::triangle_proj tp =
            mew->getTriangleProjection(triid, mew->mp, rc, mew->mp->mip->getWidth(rc), mew->mp->mip->getHeight(rc));
        if(RD.x - LU.x + 1 > textureSide)
        {
            if(tp.rd.x < LU.x + (RD.x - LU.x) / 2)
            {
                textureTiranglesMeshIds1->push_back(triid);
            }
            else
            {
                textureTiranglesMeshIds2->push_back(triid);
            }
        }
        else
        {
            if(tp.rd.y < LU.y + (RD.y - LU.y) / 2)
            {
                textureTiranglesMeshIds1->push_back(triid);
            }
            else
            {
                textureTiranglesMeshIds2->push_back(triid);
            }
        }
    }

    mv_charts_packer::chart* out1 = new mv_charts_packer::chart(segid, textureTiranglesMeshIds1->size(), mew);
    for(int i = 0; i < textureTiranglesMeshIds1->size(); i++)
    {
        out1->addTriangle((*textureTiranglesMeshIds1)[i], mew);
    }

    mv_charts_packer::chart* out2 = new mv_charts_packer::chart(segid, textureTiranglesMeshIds2->size(), mew);
    for(int i = 0; i < textureTiranglesMeshIds2->size(); i++)
    {
        out2->addTriangle((*textureTiranglesMeshIds2)[i], mew);
    }

    *_out1 = out1;
    *_out2 = out2;

    delete textureTiranglesMeshIds1;
    delete textureTiranglesMeshIds2;
}

mv_charts_packer::mv_charts_packer(int _gutter, int _textureSide, mv_mesh_unwrap* _mew)
{
    gutter = _gutter;
    mew = _mew;
    textureSide = _textureSide;

    printf("Packing charts\n");

    staticVector<pixel>* trisIdSegsId = new staticVector<pixel>(mew->tris->size());
    for(int i = 0; i < mew->tris->size(); i++)
    {
        trisIdSegsId->push_back(pixel(i, mew->univ->find(i)));
    }

    if(mew->mp->verbose)
        printf("sorting trisIdSegsId %i\n", trisIdSegsId->size());

    if(trisIdSegsId->size() >= 2)
    {
        qsort(&(*trisIdSegsId)[0], trisIdSegsId->size(), sizeof(pixel), qSortComparePixelByYAsc);
    }

    // compute number of triangles in each segment
    staticVector<int>* segsNTris = new staticVector<int>(mew->tris->size());
    segsNTris->resize_with(mew->tris->size(), -1);
    int i0 = 0;
    int nsegs = 0;
    for(int i = 0; i < trisIdSegsId->size(); i++)
    {
        if((i == trisIdSegsId->size() - 1) || ((*trisIdSegsId)[i].y != (*trisIdSegsId)[i + 1].y))
        {
            int segid = (*trisIdSegsId)[i].y;
            (*segsNTris)[segid] = i - i0 + 1;
            i0 = i;
            nsegs++;
        }
    }

    if(mew->mp->verbose)
        printf("creating charts for %i segments\n", nsegs);

    charts = new staticVector<chart*>(5 * (nsegs + 1));

    trisIdUnprocessed = new staticVector<int>(mew->tris->size());

    chart* ch = new chart((*trisIdSegsId)[0].y, (*segsNTris)[(*trisIdSegsId)[0].y], mew);
    long t1 = initEstimate();
    // TODO: performances
    for(int i = 0; i < trisIdSegsId->size(); i++)
    {
        if((ch->rc >= 0)
           //&&(ch->ntris>10)
           )
        {
            ch->addTriangle((*trisIdSegsId)[i].x, mew);
        }
        else
        {
            trisIdUnprocessed->push_back((*trisIdSegsId)[i].x);
        }

        if((i == trisIdSegsId->size() - 1) || ((*trisIdSegsId)[i].y != (*trisIdSegsId)[i + 1].y))
        {
            // if (mew->mp->verbose) printf("i %i, n %i, segid %i \n",i,trisIdSegsId->size(),segid);
            if((ch->rc >= 0)
               //&&(ch->ntris>10)
               )
            {
                // if (mew->mp->verbose) printf("createHorizons\n");
                ch->createHorizons(gutter, mew);
                charts->push_back(ch);
                // if (mew->mp->verbose) printf("done\n");
            }
            else
            {
                if(ch != nullptr)
                {
                    delete ch;
                }
                ch = nullptr;
            }
            if(i + 1 < trisIdSegsId->size())
            {
                // if (mew->mp->verbose) printf("new chart\n");
                ch = new chart((*trisIdSegsId)[i + 1].y, (*segsNTris)[(*trisIdSegsId)[i + 1].y], mew);
                // if (mew->mp->verbose) printf("done\n");
            }

            // if (mew->mp->verbose) printf("done\n");
        }

        printfEstimate(i, trisIdSegsId->size(), t1);
    }
    finishEstimate();

    if(mew->mp->verbose)
        printf("dividing charts\n");

    // divide large charts
    for(int iter = 0; iter < 2; iter++) // divide in x and y if needed in next iteration
    {
        int ncharts = charts->size();
        for(int i = 0; i < ncharts; i++)
        {
            chart* ch = (*charts)[i];
            if(ch->needsToDivide(textureSide))
            {
                chart *ch1, *ch2;
                ch->divide(mew, textureSide, &ch1, &ch2);
                ch1->createHorizons(gutter, mew);
                ch2->createHorizons(gutter, mew);

                delete(*charts)[i];
                (*charts)[i] = ch1;
                charts->push_back(ch2);
            }
        }
    }

    uhorizon = new staticVector<int>(textureSide);

    if(mew->mp->verbose)
        printf("sorting charts\n");

    if(charts->size() >= 2)
    {
        qsort(&(*charts)[0], charts->size(), sizeof(chart*), qSortCompareChartAsc);
    }

    delete trisIdSegsId;
    delete segsNTris;
}

mv_charts_packer::~mv_charts_packer()
{
    for(int i = 0; i < charts->size(); i++)
    {
        delete(*charts)[i];
    }
    delete charts;

    delete trisIdUnprocessed;
    delete uhorizon;
}

int mv_charts_packer::getChartMinDAtPoistion(int position, int chartid)
{
    int chw = (*charts)[chartid]->uhorizon->size();
    int mind = std::numeric_limits<int>::max();
    for(int i = 0; i < chw; i++)
    {
        mind = std::min(mind, (*uhorizon)[position + i] - (*(*charts)[chartid]->dhorizon)[i]);
    }
    return mind;
}

void mv_charts_packer::showHorizon()
{
    int wmap = textureSide;
    int hmap = textureSide;
    float* data = new float[wmap * hmap];
    for(int i = 0; i < wmap * hmap; i++)
    {
        data[i] = 0.0f;
    }
    for(int i = 0; i < wmap; i++)
    {
        data[i * hmap + (*uhorizon)[i]] = 1.0f;
    }
    showImageOpenCV(data, wmap, hmap, 0.0f, 1.0f);
    delete[] data;
}

int mv_charts_packer::getChartWastetSpaceAtPoistion(int position, int chartid, int mind)
{
    int chw = (*charts)[chartid]->uhorizon->size();
    int ws = 0;
    for(int i = 0; i < chw; i++)
    {
        ws += (*uhorizon)[position + i] - (*(*charts)[chartid]->dhorizon)[i] - mind;
    }
    return ws;
}

int mv_charts_packer::getMinimalWastetSpace(pixel& chartPosInAtlas, int chartid)
{
    if((*charts)[chartid]->atlasid != -1)
    {
        return -1;
    }

    int chw = (*charts)[chartid]->uhorizon->size();
    int bestws = std::numeric_limits<int>::max();
    int bestmind = -1;
    int besti = -1;

    int i = 0;
    while(i < textureSide - chw)
    {
        int mind = getChartMinDAtPoistion(i, chartid);
        if(mind >= 0)
        {
            int ws = getChartWastetSpaceAtPoistion(i, chartid, mind);
            if(ws < bestws)
            {
                bestws = ws;
                besti = i;
                bestmind = mind;
            }
        }
        i++;
    }

    if(besti >= 0)
    {
        chartPosInAtlas.x = besti;
        chartPosInAtlas.y = bestmind;
        return bestws;
    }

    return -1;
}

int mv_charts_packer::getMaximalMind(pixel& chartPosInAtlas, int chartid)
{
    if((*charts)[chartid]->atlasid != -1)
    {
        return -1;
    }

    int chw = (*charts)[chartid]->uhorizon->size();
    int bestws = -1;
    int bestmind = -1;
    int besti = -1;

    int i = 0;
    while(i < textureSide - chw)
    {
        int mind = getChartMinDAtPoistion(i, chartid);
        if(mind >= 0)
        {
            if(mind > bestmind)
            {
                int ws = getChartWastetSpaceAtPoistion(i, chartid, mind);
                bestws = ws;
                besti = i;
                bestmind = mind;
            }
        }
        i++;
    }

    if(besti >= 0)
    {
        chartPosInAtlas.x = besti;
        chartPosInAtlas.y = bestmind;
        return bestws;
    }

    return -1;
}

void mv_charts_packer::addChartToAtlas(pixel& chartPosInAtlas, int chartid, int atlasid)
{
    int chw = (*charts)[chartid]->uhorizon->size();
    for(int i = 0; i < chw; i++)
    {
        (*uhorizon)[chartPosInAtlas.x + i] = (*(*charts)[chartid]->uhorizon)[i] + chartPosInAtlas.y;
    }
    //(*charts)[chartid]->showChart();
    (*charts)[chartid]->atlasid = atlasid;
}

staticVector<voxel>* mv_charts_packer::pack(int atlasid)
{
    uhorizon->resize_with(textureSide, textureSide - 1);
    // showHorizon();

    staticVector<voxel>* atlasCharts = new staticVector<voxel>(charts->size());

    // TODO: performances
    bool ok = true;
    while(ok)
    {
        int minws = std::numeric_limits<int>::max();
        int minchartid = -1;
        pixel minchartPosInAtlas;

        int chartid = 0;
        while((chartid < charts->size()) && (minchartid == -1))
        // while (chartid<charts->size())
        {
            pixel chartPosInAtlas;
            // int ws = getMinimalWastetSpace(chartPosInAtlas, chartid);
            int ws = getMaximalMind(chartPosInAtlas, chartid);
            if((ws >= 0) && (ws < minws))
            {
                minws = ws;
                minchartid = chartid;
                minchartPosInAtlas = chartPosInAtlas;
            }
            chartid++;
        }

        if(minchartid != -1)
        {
            addChartToAtlas(minchartPosInAtlas, minchartid, atlasid);
            atlasCharts->push_back(voxel(minchartid, minchartPosInAtlas.x, minchartPosInAtlas.y));
        }
        else
        {
            ok = false;
        }

        // if (atlasid>=13) {
        // drawUVAtlas(atlasCharts);
        // showHorizon();
        //};
    }

    // drawUVAtlas(atlasCharts,atlasid);

    return atlasCharts;
}

void mv_charts_packer::drawUVAtlas(staticVector<voxel>* atlas, int atlasid)
{
    IplImage* img = cvCreateImage(cvSize(textureSide, textureSide), IPL_DEPTH_8U, 3);

    CvScalar colorWhite;
    colorWhite.val[0] = 255;
    colorWhite.val[1] = 255;
    colorWhite.val[2] = 255;
    CvScalar colorGreen;
    colorGreen.val[0] = 0;
    colorGreen.val[1] = 255;
    colorGreen.val[2] = 0;

    for(int x = 0; x < textureSide; x++)
    {
        for(int y = 0; y < textureSide; y++)
        {
            CvScalar c;
            c.val[0] = 0;
            c.val[1] = 0;
            c.val[2] = 0;
            cvSet2D(img, y, x, c);
        }
    }

    for(int i = 0; i < atlas->size(); i++)
    {
        int idchart = (*atlas)[i].x;
        pixel chartLU = pixel((*atlas)[i].y, (*atlas)[i].z);
        mv_charts_packer::chart* ch = (*charts)[idchart];
        for(int j = 0; j < ch->textureTiranglesMeshIds->size(); j++)
        {
            CvScalar color = colorWhite;
            if(i == atlas->size() - 1)
            {
                color = colorGreen;
            }
            int triid = (*ch->textureTiranglesMeshIds)[j];
            mv_mesh::triangle_proj tp =
                mew->getTriangleProjection(triid, mew->mp, ch->rc, mew->mp->mip->getWidth(ch->rc), mew->mp->mip->getHeight(ch->rc));
            mv2DTriangle t;
            for(int j = 0; j < 3; j++)
            {
                t.pts[j].x = tp.tp2ds[j].x - ((float)ch->LU.x + gutter) + (float)chartLU.x + gutter;
                t.pts[j].y = tp.tp2ds[j].y - ((float)ch->LU.y + gutter) + (float)chartLU.y + gutter;
            }
            CvPoint pt1, pt2;
            pt1.x = (int)(t.pts[0].x + 0.5f);
            pt1.y = (int)(t.pts[0].y + 0.5f);
            pt2.x = (int)(t.pts[1].x + 0.5f);
            pt2.y = (int)(t.pts[1].y + 0.5f);
            cvLine(img, pt1, pt2, color);
            pt1.x = (int)(t.pts[1].x + 0.5f);
            pt1.y = (int)(t.pts[1].y + 0.5f);
            pt2.x = (int)(t.pts[2].x + 0.5f);
            pt2.y = (int)(t.pts[2].y + 0.5f);
            cvLine(img, pt1, pt2, color);
            pt1.x = (int)(t.pts[2].x + 0.5f);
            pt1.y = (int)(t.pts[2].y + 0.5f);
            pt2.x = (int)(t.pts[0].x + 0.5f);
            pt2.y = (int)(t.pts[0].y + 0.5f);
            cvLine(img, pt1, pt2, color);
        }
    }

    for(int i = 0; i < textureSide; i++)
    {
        CvScalar c;
        c.val[0] = 255;
        c.val[1] = 0;
        c.val[2] = 0;
        cvSet2D(img, (*uhorizon)[i], i, c);
    }

    if(atlas->size() > 0)
    {
        mv_charts_packer::chart* lch = (*charts)[(*atlas)[atlas->size() - 1].x];
        int wmap = lch->RD.x - lch->LU.x + 1;
        for(int i = 0; i < wmap; i++)
        {
            CvScalar c;
            c.val[0] = 0;
            c.val[1] = 0;
            c.val[2] = 255;
            cvSet2D(img, (*lch->uhorizon)[i] + (*atlas)[atlas->size() - 1].z, i + (*atlas)[atlas->size() - 1].y, c);

            c.val[0] = 255;
            c.val[1] = 0;
            c.val[2] = 0;
            cvSet2D(img, (*lch->dhorizon)[i] + (*atlas)[atlas->size() - 1].z, i + (*atlas)[atlas->size() - 1].y, c);
        }
    }

    std::string fileNameStrOut = mew->mp->mip->newDir + "atlas" + num2strThreeDigits(atlasid) + ".jpg";
    if(cvSaveImage(fileNameStrOut.c_str(), img) == 0)
        printf("Could not save: %s\n", fileNameStrOut.c_str());
    // cvShowImage("Atlas img", img);
    // cvWaitKey();
    cvReleaseImage(&img);
}
