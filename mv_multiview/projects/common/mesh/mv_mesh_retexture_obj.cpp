#include "mv_mesh_retexture_obj.h"

#include "stdafx.h"
#include "mv_mesh_uvatlas.h"
#include "structures/mv_geometry.h"
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <map>
#include <set>

staticVector<point2d>* getObjTexturePixelsForObjTriangleIdInTriBarycCoord(point2d* pixs)
{
    pixel LU, RD;
    LU.x = (int)(std::min(std::min(pixs[0].x, pixs[1].x), pixs[2].x));
    LU.y = (int)(std::min(std::min(pixs[0].y, pixs[1].y), pixs[2].y));
    RD.x = (int)(std::max(std::max(pixs[0].x, pixs[1].x), pixs[2].x)) + 1;
    RD.y = (int)(std::max(std::max(pixs[0].y, pixs[1].y), pixs[2].y)) + 1;

    pixel dim = RD - LU;
    staticVector<point2d>* out = new staticVector<point2d>((dim.x + 1) * (dim.y + 1));

    auto checkCorners = [&](int x, int y, point2d* pixs) -> bool
    {
        for(int i = 0; i <= 1; ++i)
        {
            for(int j=0; j <= 1; ++j)
            {
                point2d pix(x + i, y + j);
                point2d pixCornerBarycUv = computeBarycentricCoordinates(pixs[0], pixs[1], pixs[2], pix);
                if(isPointInTriangle(pixCornerBarycUv))
                {
                    return true;
                }
            }
        }
        return false;
    };

    const bool tolerant = false; // TODO: still excluding some subpixelic triangles

    for(int y = LU.y; y <= RD.y; y++)
    {
        for(int x = LU.x; x <= RD.x; x++)
        {
            point2d pix(x + 0.5f, y + 0.5f);
            point2d pixBarycUv = computeBarycentricCoordinates(pixs[0], pixs[1], pixs[2], pix);
            if(!tolerant)
            {
                if(isPointInTriangle(pixBarycUv))
                    out->push_back(pixBarycUv);
            }
            else
            {
                if(dim.x <= 2 && dim.y <= 2)
                {
                    out->push_back(pixBarycUv);
                    continue;
                }
                else if(isPointInTriangle(pixBarycUv))
                {
                    out->push_back(pixBarycUv);
                }
                else if(checkCorners(x, y, pixs))
                {
                    out->push_back(pixBarycUv);
                }
            }
        }
    }
    return out;
}

point2d barycentricToCartesian(const point2d* triangle, const point2d& coords)
{
    return triangle[0] + (triangle[2] - triangle[0]) * coords.x + (triangle[1] - triangle[0]) * coords.y;
}

point3d barycentricToCartesian(const point3d* triangle, const point2d& coords)
{
    return triangle[0] + (triangle[2] - triangle[0]) * coords.x + (triangle[1] - triangle[0]) * coords.y;
}


staticVector<staticVector<int>*>* meshRetex::generateUVs(multiviewParams& mp, staticVector<staticVector<int>*>* ptsCams)
{
    if(!me)
        throw std::runtime_error("Can't generate UVs without a mesh");

    // automatic uv atlasing
    std::cout << "- generating UVs (textureSide: " << texParams.textureSide << "; padding: " << texParams.padding << ") " << std::endl;
    mv_mesh_uvatlas mua(*me, mp, ptsCams, texParams.textureSide, texParams.padding);
    // create a new mesh to store data
    mv_mesh* m = new mv_mesh();
    m->pts = new staticVector<point3d>(me->pts->size());
    m->tris = new staticVector<mv_mesh::triangle>(me->tris->size());
    trisUvIds = new staticVector<voxel>(me->tris->size());
    uvCoords = new staticVector<point2d>(me->pts->size());
    _atlases.clear();
    _atlases.resize(mua.atlases().size());

    std::map<int, int> vertexCache;
    auto* updatedPointsCams = new staticVector<staticVector<int>*>(ptsCams->size());

    int atlasId = 0;
    int triangleCount = 0;

    for(auto& charts : mua.atlases())
    {
        for(auto& chart : charts)
        {
            std::map<int, int> uvCache;

            pixel offset = chart.targetLU;
            offset = offset - chart.sourceLU;

            // for each triangle in this chart
            for(size_t i = 0 ; i<chart.triangleIDs.size(); ++i)
            {
                int triangleID = chart.triangleIDs[i];
                // register triangle in corresponding atlas
                _atlases[atlasId].push_back(triangleCount);

                mv_mesh::triangle t;
                voxel triUv;
                // for each point
                for(int k = 0; k < 3; ++k)
                {
                    int pointId = (*me->tris)[triangleID].i[k];
                    // get 3d triangle points
                    point3d p = (*me->pts)[pointId];
                    point2d pix;
                    mp.getPixelFor3DPoint(&pix, p, chart.refCameraID);
                    if(!mp.isPixelInImage(pix, chart.refCameraID))
                        continue;
                    // compute the final pixel coordinates
                    point2d uvPix = (pix + point2d(offset.x, offset.y)) / (float)mua.textureSide();
                    uvPix.y = 1.0f - uvPix.y;
                    if(uvPix.x >= mua.textureSide() || uvPix.y >= mua.textureSide())
                        continue;

                    auto it = vertexCache.find(pointId);
                    int newPointIdx;
                    int uvIdx;
                    if(it == vertexCache.end())
                    {
                        m->pts->push_back(p);
                        newPointIdx = m->pts->size() - 1;
                        // map point visibilities
                        staticVector<int>* pOther = new staticVector<int>();
                        staticVector<int>* pRef = (*ptsCams)[pointId];
                        if(pRef)
                            *pOther = *pRef;
                        updatedPointsCams->push_back(pOther);
                        // update cache
                        vertexCache[pointId] = newPointIdx;
                    }
                    else
                    {
                        newPointIdx = it->second;
                    }
                    t.i[k] = newPointIdx;
                    // store uv coord and triangle mapping
                    auto uvcacheIt = uvCache.find(newPointIdx);
                    if(uvcacheIt == uvCache.end())
                    {
                        uvCoords->push_back(uvPix);
                        uvIdx = uvCoords->size() - 1;
                        uvCache[newPointIdx] = uvIdx;
                    }
                    else
                        uvIdx = uvcacheIt->second;
                    triUv.m[k] = uvIdx;

                }
                m->tris->push_back(t);
                trisUvIds->push_back(triUv);
                triangleCount++;
            }
        }
        atlasId++;
    }

    // replace internal mesh
    std::swap(me, m);
    delete m;

    return updatedPointsCams;
}

void meshRetex::generateTextures(const multiviewParams &mp, staticVector<staticVector<int> *> *ptsCams,
                                 const boost::filesystem::path &outPath)
{
    mv_images_cache imageCache(&mp, 0, false);
    for(size_t atlasID = 0; atlasID < _atlases.size(); ++atlasID)
        generateTexture(mp, ptsCams, atlasID, imageCache, outPath);
}

//// pixel coordinates mapping, between source and final image
struct PixelMapping {
    point2d coordinatesInTC;
    size_t indexInFinalImg;
};

/// accumulates colors and keeps count for providing average
struct AccuColor {
    Color colorSum;
    unsigned int count = 0;

    unsigned int add(const Color& color)
    {
        colorSum = colorSum + color;
        return ++count;
    }

    Color average() const {
        return count > 0 ? colorSum / count : colorSum;
    }
};

// alternative method to iterate over UV triangles' pixels
// wip: fails on some triangles smaller than a pixel
// #define USE_BARYCENTRIC_COORDS

void meshRetex::generateTexture(const multiviewParams& mp, staticVector<staticVector<int>*>* ptsCams,
                                size_t atlasID, mv_images_cache& imageCache, const bfs::path& outPath)
{
    if(atlasID >= _atlases.size())
        throw std::runtime_error("Invalid atlas ID " + std::to_string(atlasID));

    unsigned int textureSize = texParams.textureSide * texParams.textureSide;
    std::vector<std::vector<PixelMapping*>> perCamPixelMapping(mp.ncams);
    std::vector<int> colorIDs(textureSize, -1);

    std::cout << "Generating texture for atlas " << atlasID + 1 << "/" << _atlases.size()
              << " (" << _atlases[atlasID].size() << " triangles)" << std::endl;
    std::cout << "- pixel mapping " << std::endl;

    // iterate over atlas' triangles
    for(size_t i = 0; i < _atlases[atlasID].size(); ++i)
    {
        int triangleId = _atlases[atlasID][i];
        point2d triPixs[3];
        point3d triPts[3];
        std::set<int> triCams;
        // get corresponding 3D coordinates, UVs coordinates and report visibilities
        for(int k = 0; k < 3; k++)
        {
            const int pointIndex = (*me->tris)[triangleId].i[k];
            triPts[k] = (*me->pts)[pointIndex];                               // 3D coordinate
            const int uvPointIndex = (*trisUvIds)[triangleId].m[k];
            triPixs[k] = (*uvCoords)[uvPointIndex] * texParams.textureSide;   // UV coordinate

            // retrieve triangle visibilities (set of triangle's points visibilities)
            if((*ptsCams)[pointIndex] != nullptr)
                std::copy((*ptsCams)[pointIndex]->begin(), (*ptsCams)[pointIndex]->end(), std::inserter(triCams, triCams.end()));
        }
        if(triCams.empty()) // early-exit: triangle has no visibility
        {
            continue;
        }

#ifdef USE_BARYCENTRIC_COORDS
        // (2D) get every texture pixels contained inside the UV triangle
        staticVector<point2d>* triTexturePixels = getObjTexturePixelsForObjTriangleIdInTriBarycCoord(triPixs);

        // project in cameras seeing that triangle
        for(const point2d& coord : *triTexturePixels)
        {
            // convert barycentric coordinates to uv
            point2d pix = barycentricToCartesian(triPixs, coord);

#else
        double sa = std::max(2.0, (triPixs[1] - triPixs[0]).size() + 1);
        double sb = std::max(2.0, (triPixs[2] - triPixs[0]).size() + 1);
        point3d coeff3D_a = (triPts[1] - triPts[0]) / sa;
        point3d coeff3D_b = (triPts[2] - triPts[0]) / sb;
        point2d coeff2D_a = (triPixs[1] - triPixs[0]) / sa;
        point2d coeff2D_b = (triPixs[2] - triPixs[0]) / sb;

        // get pixels inside this triangle
        //
        // we iterate over half pixels to avoid precision issues (missing pixels in final texture)
        // if a pixel already has color info, it's skipped; impact on performance is minor
        for(double ti = 0.; ti <= sa; ti+=0.5)
        {
        for(double tj = 0.; tj <= ((sa - ti) / sa) * sb; tj+=0.5)
        {
            point2d pix = triPixs[0] + coeff2D_a * ti + coeff2D_b * tj;
#endif
            // get 1D pixel index
            pix.y = texParams.textureSide - pix.y;
            size_t xyoffset = (int)pix.y * texParams.textureSide + (int)pix.x;
            if(colorIDs[xyoffset] != -1)
                continue;
            // store color id for this index (used later for padding)
            colorIDs[xyoffset] = xyoffset;

#ifdef USE_BARYCENTRIC_COORDS
            point3d pt3d = barycentricToCartesian(triPts, coord);
#else
            point3d pt3d = triPts[0] + coeff3D_a * ti + coeff3D_b * tj;
#endif
            for(const auto& camId : triCams)
            {
                point2d pixRC;
                mp.getPixelFor3DPoint(&pixRC, pt3d, camId);
                if(!mp.isPixelInImage(pixRC, camId))
                    continue;
                // store pixel mapping
                PixelMapping* pi = new PixelMapping();
                pi->coordinatesInTC = pixRC;
                pi->indexInFinalImg = xyoffset;
                perCamPixelMapping[camId].emplace_back(pi);
            }
        }
#ifdef USE_BARYCENTRIC_COORDS
        delete triTexturePixels;
#else
        }
#endif
    }

    std::cout << "- reading pixel color" << std::endl;

    std::vector<AccuColor> perPixelColors(textureSize);
    for(size_t camId = 0; camId < perCamPixelMapping.size(); ++camId)
    {
        auto& camPixelMappings = perCamPixelMapping[camId];
        for(auto* pi : camPixelMappings)
        {
            // read from image cache
            size_t xyoffset = pi->indexInFinalImg;
            perPixelColors[xyoffset].add(imageCache.getPixelValueInterpolated(&(pi->coordinatesInTC), camId));
            delete pi;
        }
        camPixelMappings.clear();
    }
    perCamPixelMapping.clear();

    std::cout << "- edge padding (" << texParams.padding << " pixels)" << std::endl;
    // edge padding (dilate gutter)
    for(unsigned int g = 0; g < texParams.padding; ++g)
    {
        for(unsigned int y = 1; y < texParams.textureSide-1; ++y)
        {
            unsigned int yoffset = y * texParams.textureSide;
            for(unsigned int x = 1; x < texParams.textureSide-1; ++x)
            {
                unsigned int xyoffset = yoffset + x;
                if(colorIDs[xyoffset] > 0)
                    continue;
                if(colorIDs[xyoffset-1] > 0)
                {
                    colorIDs[xyoffset] = (xyoffset-1)*-1;
                    continue;
                }
                if(colorIDs[xyoffset+1] > 0)
                {
                    colorIDs[xyoffset] = (xyoffset+1)*-1;
                    continue;
                }
                if(colorIDs[xyoffset+texParams.textureSide] > 0)
                {
                    colorIDs[xyoffset] = (xyoffset+texParams.textureSide)*-1;
                    continue;
                }
                if(colorIDs[xyoffset-texParams.textureSide] > 0)
                {
                    colorIDs[xyoffset] = (xyoffset-texParams.textureSide)*-1;
                    continue;
                }
            }
        }
        for(unsigned int i=0; i < textureSize; ++i)
        {
            if(colorIDs[i] < 0)
                colorIDs[i] = colorIDs[colorIDs[i]*-1];
        }
    }

    std::cout << "- computing final (average) color" << std::endl;

    // save texture image
    IplImage* image = cvCreateImage(cvSize(texParams.textureSide, texParams.textureSide), IPL_DEPTH_8U, 3);
    for(unsigned int yp = 0; yp < texParams.textureSide; ++yp)
    {
        unsigned int yoffset = yp * texParams.textureSide;
        for(unsigned int xp = 0; xp < texParams.textureSide; ++xp)
        {
            unsigned int xyoffset = yoffset + xp;
            int colorID = colorIDs[xyoffset];
            Color color = colorID ? perPixelColors[colorID].average() : Color();
            CvScalar c;
            c.val[0] = color.b;
            c.val[1] = color.g;
            c.val[2] = color.r;
            cvSet2D(image, yp, xp, c);
        }
    }
    perPixelColors.clear();
    colorIDs.clear();

    // downscale texture if required
    if(texParams.downscale > 1)
    {
        std::cout << "- downscaling texture (" << texParams.downscale << "x)" << std::endl;
        const unsigned int dsTexSide = texParams.textureSide / texParams.downscale;
        IplImage* scaledImage = cvCreateImage(cvSize(dsTexSide, dsTexSide), IPL_DEPTH_8U, 3);
        cvResize(image, scaledImage);
        cvReleaseImage(&image);
        image = scaledImage;
    }

    std::string textureName = "texture_" + std::to_string(atlasID) + ".png";
    bfs::path texturePath = outPath / textureName;
    std::cout << "- writing texture file " << texturePath.string() << std::endl;
    if(!cvSaveImage(texturePath.string().c_str(), image))
        std::cerr << "Could not save: " << texturePath << std::endl;
    cvReleaseImage(&image);
}


void meshRetex::loadFromOBJ(const std::string& filename, bool flipNormals)
{
    // Load .obj
    if(!me->loadFromObjAscii(nmtls, &trisMtlIds,
                             &normals, &trisNormalsIds, &uvCoords, &trisUvIds,
                             filename.c_str()))
    {
        throw std::runtime_error("Unable to load: " + filename);
    }

    // Handle normals flipping
    if(flipNormals)
        me->invertTriangleOrientations();

    // Fill atlases (1 atlas per material) with corresponding rectangles
    // if no material, create only one atlas with all triangles
    _atlases.resize(std::max(1, nmtls));
    for(int triangleID = 0; triangleID < trisMtlIds->size(); triangleID++)
    {
        unsigned int atlasID = nmtls ? (*trisMtlIds)[triangleID] : 0;
        _atlases[atlasID].push_back(triangleID);
    }
}

void meshRetex::saveAsOBJ(const bfs::path& dir, const std::string& basename)
{
    std::cout << "- writing .obj and .mtl file" << std::endl;

    std::string objFilename = (dir / (basename + ".obj")).string();
    std::string mtlName = (basename + ".mtl");
    std::string mtlFilename = (dir / mtlName).string();

    // create .OBJ file
    FILE* fobj = fopen(objFilename.c_str(), "w");

    // header
    fprintf(fobj, "# \n");
    fprintf(fobj, "# Wavefront OBJ file\n");
    fprintf(fobj, "# Created with AliceVision\n");
    fprintf(fobj, "# \n");
    fprintf(fobj, "mtllib %s\n\n", mtlName.c_str());
    fprintf(fobj, "g TexturedMesh\n");

    // write vertices
    auto vertices = me->pts;
    for(int i = 0; i < vertices->size(); ++i)
        fprintf(fobj, "v %f %f %f\n", (*vertices)[i].x, (*vertices)[i].y, (*vertices)[i].z);

    // write UV coordinates
    for(int i=0; i < uvCoords->size(); ++i)
        fprintf(fobj, "vt %f %f\n", (*uvCoords)[i].x, (*uvCoords)[i].y);

    // write faces per texture atlas
    for(size_t atlasID=0; atlasID < _atlases.size(); ++atlasID)
    {
        fprintf(fobj, "usemtl TextureAtlas_%i\n", atlasID);
        for(const auto triangleID : _atlases[atlasID])
        {
            // vertex IDs
            int vertexID1 = (*me->tris)[triangleID].i[0];
            int vertexID2 = (*me->tris)[triangleID].i[1];
            int vertexID3 = (*me->tris)[triangleID].i[2];

            int uvID1 = (*trisUvIds)[triangleID].m[0];
            int uvID2 = (*trisUvIds)[triangleID].m[1];
            int uvID3 = (*trisUvIds)[triangleID].m[2];

            fprintf(fobj, "f %i/%i %i/%i %i/%i\n", vertexID1 + 1, uvID1 + 1, vertexID2 + 1, uvID2 + 1, vertexID3 + 1, uvID3 + 1); // indexed from 1
        }
    }
    fclose(fobj);
    std::cout << "OBJ: " << objFilename << std::endl;

    // create .MTL material file
    FILE* fmtl = fopen(mtlFilename.c_str(), "w");

    // header
    fprintf(fmtl, "# \n");
    fprintf(fmtl, "# Wavefront material file\n");
    fprintf(fmtl, "# Created with AliceVision\n");
    fprintf(fmtl, "# \n\n");

    // for each atlas, create a new material with associated texture
    for(size_t atlasID=0; atlasID < _atlases.size(); ++atlasID)
    {
        std::string textureName = "texture_" + std::to_string(atlasID) + ".png";
        fprintf(fmtl, "\n");
        fprintf(fmtl, "newmtl TextureAtlas_%i\n", atlasID);
        fprintf(fmtl, "Ka  0.6 0.6 0.6\n");
        fprintf(fmtl, "Kd  0.6 0.6 0.6\n");
        fprintf(fmtl, "Ks  0.0 0.0 0.0\n");
        fprintf(fmtl, "d  1.0\n");
        fprintf(fmtl, "Ns  0.0\n");
        fprintf(fmtl, "illum 2\n");
        fprintf(fmtl, "map_Kd %s\n", textureName.c_str());
    }
    fclose(fmtl);
    std::cout << "MTL: " << mtlFilename << std::endl;
}
