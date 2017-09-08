#include "mv_filesio.h"
#include "mv_multiview_params.h"
#include "stdafx.h"



#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

bool FileExists(const std::string& filePath)
{
    return boost::filesystem::exists(filePath);
}

bool FolderExists(const std::string& folderPath)
{
    return boost::filesystem::is_directory(folderPath);
}

std::string mv_getFileNamePrefixRcTc(multiviewInputParams* mip, int rc, int tc)
{
    return mip->mvDir + mip->prefix + num2strFourDecimal(rc) + "_" + num2strFourDecimal(tc);
}

std::string mv_getFileNamePrefix(multiviewInputParams* mip, int index)
{
    return mip->mvDir + mip->prefix + num2strFourDecimal(index);
}

std::string mv_getFileName(multiviewInputParams* mip, int index, int mv_file_type)
{
    std::string fileName = mv_getFileNamePrefix(mip, index);

    if(mv_file_type == mip->MV_FILE_TYPE_P)
    {
        fileName += "_P.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_K)
    {
        fileName += "_K.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_iK)
    {
        fileName += "_iK.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_R)
    {
        fileName += "_R.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_iR)
    {
        fileName += "_iR.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_C)
    {
        fileName += "_C.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_iP)
    {
        fileName += "_iP.txt";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_har)
    {
        fileName += "_har.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_prematched)
    {
        fileName += "_prematched.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_seeds)
    {
        fileName += "_seeds.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_growed)
    {
        fileName += "_growed.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_occMap)
    {
        fileName += "_occMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_nearMap)
    {
        fileName += "_nearMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_op)
    {
        fileName += "_op.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_wshed)
    {
        fileName += "_wshed.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_seeds_prm)
    {
        fileName += "_seeds_prm.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_seeds_flt)
    {
        fileName += "_seeds_flt.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_img)
    {
        fileName += "_img.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_imgT)
    {
        fileName += "_imgT.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_seeds_seg)
    {
        fileName += "_seeds_seg.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_graphCutMap)
    {
        fileName += "_graphCutMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_graphCutPts)
    {
        fileName += "_graphCutPts.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_growedMap)
    {
        fileName += "_growedMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_agreedMap)
    {
        fileName += "_agreedMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_agreedPts)
    {
        fileName += "_agreedPts.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_refinedMap)
    {
        fileName += "_refinedMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_seeds_sfm)
    {
        fileName += "_seeds_sfm.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_radial_disortion)
    {
        fileName += "_rd.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_graphCutMesh)
    {
        fileName += "_graphCutMesh.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_agreedMesh)
    {
        fileName += "_agreedMesh.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_nearestAgreedMap)
    {
        fileName += "_nearestAgreedMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_segPlanes)
    {
        fileName += "_segPlanes.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_agreedVisMap)
    {
        fileName += "_agreedVisMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_diskSizeMap)
    {
        fileName += "_diskSizeMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_depthMap)
    {
        fileName += "_depthMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_simMap)
    {
        fileName += "_simMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_mapPtsTmp)
    {
        fileName += "_mapPts.tmp";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_mapPtsSimsTmp)
    {
        fileName += "_mapPtsSims.tmp";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_depthMapInfo)
    {
        fileName += "_depthMapInfo.tmp";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_camMap)
    {
        fileName += "_camMap.bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_nmodMap)
    {
        fileName += "_nmodMap.bin";
    }

    return fileName;
}

std::string mv_getFileName(multiviewInputParams* mip, int index, int mv_file_type, int scale)
{
    if(scale == 0)
    {
        return mv_getFileName(mip, index, mv_file_type);
    }

    std::string fileName = mv_getFileNamePrefix(mip, index);

    if(mv_file_type == mip->MV_FILE_TYPE_depthMap)
    {
        fileName += "_depthMap_scale" + num2str(scale) + ".bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_simMap)
    {
        fileName += "_simMap_scale" + num2str(scale) + ".bin";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_mapPtsTmp)
    {
        fileName += "_mapPts_scale" + num2str(scale) + ".tmp";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_depthMapInfo)
    {
        fileName += "_depthMapInfo_scale" + num2str(scale) + ".tmp";
    }
    if(mv_file_type == mip->MV_FILE_TYPE_camMap)
    {
        fileName += "_camMap_scale" + num2str(scale) + ".bin";
    }

    return fileName;
}

FILE* mv_openFile(multiviewInputParams* mip, int index, int mv_file_type, const char* readWrite)
{
    std::string fileName = mv_getFileName(mip, index, mv_file_type);

    FILE* out = fopen(fileName.c_str(), readWrite);

    /*
    if (out==NULL) {
            printf("file %s does not exists!",  fileName.c_str());
    };
    */

    return out;
}

point3d load3x1MatrixFromFile(FILE* fi)
{
    point3d m;

    float a, b, c;
    fscanf(fi, "%f \n", &a);
    fscanf(fi, "%f \n", &b);
    fscanf(fi, "%f \n", &c);

    m.x = a;
    m.y = b;
    m.z = c;

    return m;
}

matrix3x3 load3x3MatrixFromFile(FILE* fi)
{
    matrix3x3 m;

    // M[col*3 + row]
    float a, b, c;
    fscanf(fi, "%f %f %f \n", &a, &b, &c);
    m.m11 = a;
    m.m12 = b;
    m.m13 = c;
    fscanf(fi, "%f %f %f \n", &a, &b, &c);
    m.m21 = a;
    m.m22 = b;
    m.m23 = c;
    fscanf(fi, "%f %f %f \n", &a, &b, &c);
    m.m31 = a;
    m.m32 = b;
    m.m33 = c;

    return m;
}

matrix3x4 load3x4MatrixFromFile(FILE* fi)
{
    matrix3x4 m;

    // M[col*3 + row]
    float a, b, c, d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m11 = a;
    m.m12 = b;
    m.m13 = c;
    m.m14 = d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m21 = a;
    m.m22 = b;
    m.m23 = c;
    m.m24 = d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m31 = a;
    m.m32 = b;
    m.m33 = c;
    m.m34 = d;

    return m;
}

point3d loadPoint3dFromFile(FILE* fi)
{
    point3d m;

    // M[col*3 + row]
    float a, b, c;
    fscanf(fi, "%f %f %f %*f \n", &a, &b, &c);
    m.x = a;
    m.y = b;
    m.z = c;

    return m;
}

int get2dPointsNum(int imgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, imgFileId, mip->MV_FILE_TYPE_har, "r");

    if(f == nullptr)
    {
        return 0;
    }

    int n = 0;
    while(feof(f) == 0)
    {
        float a, b;
        fscanf(f, "%f %f \n", &a, &b);
        n++;
    }

    fclose(f);

    return n;
}

int load2dPoints(point2d** _out, int imgFileId, multiviewInputParams* mip)
{
    point2d* out;
    int n = get2dPointsNum(imgFileId, mip);
    if(n == 0)
    {
        out = new point2d[1];
    }
    else
    {
        out = new point2d[n];

        FILE* f = mv_openFile(mip, imgFileId, mip->MV_FILE_TYPE_har, "r");

        int i = 0;
        while(feof(f) == 0)
        {
            float a, b;
            fscanf(f, "%f %f \n", &a, &b);
            out[i] = point2d((float)a, (float)b);
            i++;
        }

        fclose(f);
    }

    *_out = out;

    return n;
}

staticVector<point2d>* load2dPoints(int imgFileId, multiviewInputParams* mip)
{
    int n = get2dPointsNum(imgFileId, mip);

    staticVector<point2d>* out = new staticVector<point2d>(std::max(n, 1));

    if(n > 0)
    {
        FILE* f = mv_openFile(mip, imgFileId, mip->MV_FILE_TYPE_har, "r");
        while(feof(f) == 0)
        {
            float a, b;
            fscanf(f, "%f %f \n", &a, &b);
            out->push_back(point2d((float)a, (float)b));
        }
        fclose(f);
    }

    return out;
}

void load2dPoints(staticVector<point2d>* out, int imgFileId, multiviewInputParams* mip)
{
    out->resize(0);
    FILE* f = mv_openFile(mip, imgFileId, mip->MV_FILE_TYPE_har, "r");
    while(feof(f) == 0)
    {
        float a, b;
        fscanf(f, "%f %f \n", &a, &b);
        out->push_back(point2d((float)a, (float)b));
    }
    fclose(f);
}

/*
bool saveImgAsImg(const std::string& fileName, const std::string& fileNameT, int imgFileId, multiviewInputParams* mip)
{
    // check if exists silhouette image
    bool usesil = false;
    IplImage* simg = nullptr;
    std::string fileNameSilStr = mv_getFileNamePrefix(mip, imgFileId) + "_sil." + mip->imageExt;
    if(FileExists(fileNameSilStr))
    {
        simg = cvLoadImage(fileNameSilStr.c_str());
        usesil = true;
    }

    std::string fileNameOrigStr = mv_getFileNamePrefix(mip, imgFileId) + "." + mip->imageExt;

    IplImage* cimg = cvLoadImage(fileNameOrigStr.c_str());
    if(cimg != nullptr)
    {
        // check image size...
        if((mip->getWidth(rc) == cimg->width) && (mip->getHeight(rc) == cimg->height))
        {
            int w = mip->getWidth(rc);
            int h = mip->getHeight(rc);

            unsigned char* imgArr = new unsigned char[mip->imp.im_size];
            unsigned char* imgTArr = new unsigned char[mip->imp.im_size];

            IplImage* gimg = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
            cvCvtColor(cimg, gimg, CV_BGR2GRAY);

            for(int x = 0; x < mip->getWidth(rc); x++)
            {
                for(int y = 0; y < mip->getHeight(rc); y++)
                {
                    if(usesil)
                    {
                        CvScalar s;
                        s = cvGet2D(simg, y, x);
                        if(s.val[0] == 0)
                        {
                            imgArr[x * h + y] = 0;
                            imgTArr[y * w + x] = 0;
                        }
                        else
                        {
                            s = cvGet2D(gimg, y, x);
                            imgArr[x * h + y] = s.val[0];
                            imgTArr[y * w + x] = s.val[0];
                        }
                    }
                    else
                    {
                        CvScalar s;
                        s = cvGet2D(gimg, y, x);
                        imgArr[x * h + y] = s.val[0];
                        imgTArr[y * w + x] = s.val[0];
                    }
                }
            }

            // save gegs
            FILE* f = fopen(fileName.c_str(), "wb");
            if(f == nullptr)
            {
                printf("???\n");
            }
            else
            {
                fwrite(&imgArr[0], sizeof(unsigned char), mip->imp.im_size, f);
                fclose(f);
            }

            f = fopen(fileNameT.c_str(), "wb");
            if(f == nullptr)
            {
                printf("???\n");
            }
            else
            {
                fwrite(&imgTArr[0], sizeof(unsigned char), mip->imp.im_size, f);
                fclose(f);
            }

            delete[] imgArr;
            delete[] imgTArr;
            cvReleaseImage(&gimg);
            cvReleaseImage(&cimg);
        }
        else
        {
            printf("!width, height \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        printf("!LoadBitmapA(g_streamFactory, %s, bmp) \n", fileNameOrigStr.c_str());
        exit(EXIT_FAILURE);
    }

    if(usesil)
    {
        cvReleaseImage(&simg);
    }

    return usesil;
}


void downscaleImage(const std::string& fileName, const std::string& fileNameT, int imgFileId, multiviewInputParams* mip,
                    int scaleFactor)
{
    // check if exists silhouette image
    std::string fileNameOrigStr = mv_getFileNamePrefix(mip, imgFileId) + "." + mip->imageExt;
    IplImage* cimg = cvLoadImage(fileNameOrigStr.c_str());
    if(cimg != nullptr)
    {
        // check image size...
        if((mip->getWidth(rc) == cimg->width) && (mip->getHeight(rc) == cimg->height))
        {
            int w = mip->getWidth(rc) / scaleFactor;
            int h = mip->getHeight(rc) / scaleFactor;

            IplImage* bmpr = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
            cvResize(cimg, bmpr);

            IplImage* gimg = cvCreateImage(cvSize(bmpr->width, bmpr->height), IPL_DEPTH_8U, 1);
            cvCvtColor(bmpr, gimg, CV_BGR2GRAY);

            unsigned char* imgArr = new unsigned char[w * h];
            unsigned char* imgTArr = new unsigned char[w * h];

            for(int x = 0; x < w; x++)
            {
                for(int y = 0; y < h; y++)
                {
                    CvScalar s;
                    s = cvGet2D(gimg, y, x);
                    imgArr[x * h + y] = s.val[0];
                    imgTArr[y * w + x] = s.val[0];
                }
            }

            // save gegs
            FILE* f = fopen(fileName.c_str(), "wb");
            if(f == nullptr)
            {
                printf("???\n");
            }
            else
            {
                fwrite(&imgArr[0], sizeof(unsigned char), w * h, f);
                fclose(f);
            }

            f = fopen(fileNameT.c_str(), "wb");
            if(f == nullptr)
            {
                printf("???\n");
            }
            else
            {
                fwrite(&imgTArr[0], sizeof(unsigned char), w * h, f);
                fclose(f);
            }

            delete[] imgArr;
            delete[] imgTArr;
            cvReleaseImage(&gimg);
            cvReleaseImage(&cimg);
            cvReleaseImage(&bmpr);
        }
        else
        {
            printf("!width, height \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        printf("!LoadBitmapA(g_streamFactory, fileNameStr.c_str(), bmp) \n");
        exit(EXIT_FAILURE);
    }
}

void blurImage(const std::string& fileName, const std::string& fileNameT, int imgFileId, multiviewInputParams* mip,
               int scaleFactor, int blurWin)
{
    // check if exists silhouette image
    std::string fileNameOrigStr = mv_getFileNamePrefix(mip, imgFileId) + "." + mip->imageExt;
    IplImage* cimg = cvLoadImage(fileNameOrigStr.c_str());
    if(cimg != nullptr)
    {
        // check image size...
        if((mip->getWidth(rc) == cimg->width) && (mip->getHeight(rc) == cimg->height))
        {
            int w = mip->getWidth(rc) / scaleFactor;
            int h = mip->getHeight(rc) / scaleFactor;

            IplImage* bmpr = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
            cvResize(cimg, bmpr);
            cvSmooth(bmpr, bmpr, CV_BLUR, blurWin, blurWin);

            IplImage* gimg = cvCreateImage(cvSize(bmpr->width, bmpr->height), IPL_DEPTH_8U, 1);
            cvCvtColor(bmpr, gimg, CV_BGR2GRAY);

            unsigned char* imgArr = new unsigned char[w * h];
            unsigned char* imgTArr = new unsigned char[w * h];

            for(int x = 0; x < w; x++)
            {
                for(int y = 0; y < h; y++)
                {
                    CvScalar s;
                    s = cvGet2D(gimg, y, x);
                    imgArr[x * h + y] = s.val[0];
                    imgTArr[y * w + x] = s.val[0];
                }
            }

            // save gegs
            FILE* f = fopen(fileName.c_str(), "wb");
            if(f == nullptr)
            {
                printf("???\n");
            }
            else
            {
                fwrite(&imgArr[0], sizeof(unsigned char), w * h, f);
                fclose(f);
            }

            f = fopen(fileNameT.c_str(), "wb");
            if(f == nullptr)
            {
                printf("???\n");
            }
            else
            {
                fwrite(&imgTArr[0], sizeof(unsigned char), w * h, f);
                fclose(f);
            }

            delete[] imgArr;
            delete[] imgTArr;
            cvReleaseImage(&gimg);
            cvReleaseImage(&cimg);
            cvReleaseImage(&bmpr);
        }
        else
        {
            printf("!width, height \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        printf("!LoadBitmapA(g_streamFactory, fileNameStr.c_str(), bmp) \n");
        exit(EXIT_FAILURE);
    }
}
*/
void memcpyRGBImageFromFileToArr(int camId, rgb* imgArr, int imgFileId, multiviewInputParams* mip, bool transpose, int scaleFactor,
                                 int bandType)
{
    std::string fileNameOrigStr = mv_getFileNamePrefix(mip, imgFileId) + "._c.png";
    memcpyRGBImageFromFileToArr(camId, imgArr, fileNameOrigStr, mip, transpose, scaleFactor, bandType);
}

void memcpyRGBImageFromFileToArr(int camId, rgb* imgArr, const std::string& fileNameOrigStr, multiviewInputParams* mip,
                                 bool transpose, int scaleFactor, int bandType)
{
    int w = mip->getWidth(camId) / std::max(scaleFactor, 1);
    int h = mip->getHeight(camId) / std::max(scaleFactor, 1);

    IplImage* cimg = cvLoadImage(fileNameOrigStr.c_str());
    if(cimg != nullptr)
    {
        // check image size...
        if((mip->getWidth(camId) == cimg->width) && (mip->getHeight(camId) == cimg->height))
        {
            if(scaleFactor > 1)
            {
                IplImage* bmpr = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
                cvResize(cimg, bmpr);
                cvReleaseImage(&cimg);
                cimg = bmpr;
            }

            if(bandType == 1)
            {

                // IplImage* cimg1=cvCreateImage(cvSize(cimg->width,cimg->height),IPL_DEPTH_8U,3);
                // cvSmooth(cimg1,cimg,CV_BILATERAL,11,0,10.0,5.0);
                // cvReleaseImage(&cimg);
                // cimg=cimg1;
                cvSmooth(cimg, cimg, CV_GAUSSIAN, 11);
            }

            if(bandType == 2)
            {
                IplImage* bmps = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
                cvSmooth(cimg, bmps, CV_GAUSSIAN, 11);
                for(int y = 0; y < h; y++)
                {
                    for(int x = 0; x < w; x++)
                    {
                        CvScalar s1, s2;
                        s1 = cvGet2D(cimg, y, x);
                        s2 = cvGet2D(bmps, y, x);
                        s1.val[0] = s1.val[0] - s2.val[0];
                        s1.val[1] = s1.val[1] - s2.val[1];
                        s1.val[2] = s1.val[2] - s2.val[2];
                        cvSet2D(cimg, y, x, s1);
                    }
                }
                cvReleaseImage(&bmps);
            }

            for(int y = 0; y < h; y++)
            {
                for(int x = 0; x < w; x++)
                {
                    // CvScalar s;
                    // s=cvGet2D(cimg,y,x);
                    uchar B = ((uchar*)(cimg->imageData + y * cimg->widthStep))[x * cimg->nChannels + 0];
                    uchar G = ((uchar*)(cimg->imageData + y * cimg->widthStep))[x * cimg->nChannels + 1];
                    uchar R = ((uchar*)(cimg->imageData + y * cimg->widthStep))[x * cimg->nChannels + 2];

                    if(transpose)
                    {
                        imgArr[y * w + x].b = B;
                        imgArr[y * w + x].g = G;
                        imgArr[y * w + x].r = R;
                    }
                    else
                    {
                        imgArr[x * h + y].b = B;
                        imgArr[x * h + y].g = G;
                        imgArr[x * h + y].r = R;
                    }
                }
            }

            cvReleaseImage(&cimg);
        }
        else
        {
            printf("!width, height \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        printf("WARNING file %s does not exists!\n", fileNameOrigStr.c_str());
        exit(EXIT_FAILURE);
    }
}

void memcpyRGBImageFromFileToArr(int camId, unsigned char* imgArrR, unsigned char* imgArrG, unsigned char* imgArrB, int imgFileId,
                                 multiviewInputParams* mip, bool transpose, int scaleFactor, int bandType)
{
    std::string fileNameOrigStr = mv_getFileNamePrefix(mip, imgFileId) + "._c.png";
    memcpyRGBImageFromFileToArr(camId, imgArrR, imgArrG, imgArrB, fileNameOrigStr, mip, transpose, scaleFactor, bandType);
}

void memcpyRGBImageFromFileToArr(int camId, unsigned char* imgArrR, unsigned char* imgArrG, unsigned char* imgArrB,
                                 const std::string& fileNameOrigStr, multiviewInputParams* mip, bool transpose,
                                 int scaleFactor, int bandType)
{
    int w = mip->getWidth(camId) / std::max(scaleFactor, 1);
    int h = mip->getHeight(camId) / std::max(scaleFactor, 1);

    IplImage* cimg = cvLoadImage(fileNameOrigStr.c_str());
    if(cimg != nullptr)
    {
        // check image size...
        if((mip->getWidth(camId) == cimg->width) && (mip->getHeight(camId) == cimg->height))
        {
            if(scaleFactor > 1)
            {
                IplImage* bmpr = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
                cvResize(cimg, bmpr);
                cvReleaseImage(&cimg);
                cimg = bmpr;
            }

            if(bandType == 1)
            {
                cvSmooth(cimg, cimg, CV_GAUSSIAN, 11);
            }

            if(bandType == 2)
            {
                IplImage* bmps = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
                cvSmooth(cimg, bmps, CV_GAUSSIAN, 11);
                for(int y = 0; y < h; y++)
                {
                    for(int x = 0; x < w; x++)
                    {
                        CvScalar s1, s2;
                        s1 = cvGet2D(cimg, y, x);
                        s2 = cvGet2D(bmps, y, x);
                        s1.val[0] = s1.val[0] - s2.val[0];
                        s1.val[1] = s1.val[1] - s2.val[1];
                        s1.val[2] = s1.val[2] - s2.val[2];
                        cvSet2D(cimg, y, x, s1);
                    }
                }
                cvReleaseImage(&bmps);
            }

            for(int y = 0; y < h; y++)
            {
                for(int x = 0; x < w; x++)
                {
                    CvScalar s;
                    s = cvGet2D(cimg, y, x);
                    if(transpose)
                    {
                        imgArrR[y * w + x] = s.val[0];
                        imgArrG[y * w + x] = s.val[1];
                        imgArrB[y * w + x] = s.val[2];
                    }
                    else
                    {
                        imgArrR[x * h + y] = s.val[0];
                        imgArrG[x * h + y] = s.val[1];
                        imgArrB[x * h + y] = s.val[2];
                    }
                }
            }

            cvReleaseImage(&cimg);
        }
        else
        {
            printf("!width, height \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        printf("WARNING file %s does not exists!\n", fileNameOrigStr.c_str());
        exit(EXIT_FAILURE);
    }
}

void memcpyGrayImageFromFileToArr(int camId, unsigned char* imgArr, int imgFileId, multiviewInputParams* mip, bool transpose,
                                  int scaleFactor, int bandType)
{
    int w = mip->getWidth(camId) / std::max(scaleFactor, 1);
    int h = mip->getHeight(camId) / std::max(scaleFactor, 1);

    std::string fileNameOrigStr = mv_getFileNamePrefix(mip, imgFileId) + "._g.png";
    IplImage* gimg = cvLoadImage(fileNameOrigStr.c_str());
    if(gimg != nullptr)
    {
        // check image size...
        if((mip->getWidth(camId) == gimg->width) && (mip->getHeight(camId) == gimg->height))
        {
            if(scaleFactor > 1)
            {
                IplImage* bmpr = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
                cvResize(gimg, bmpr);
                cvReleaseImage(&gimg);
                gimg = bmpr;
            }

            if(bandType == 1)
            {
                cvSmooth(gimg, gimg, CV_GAUSSIAN, 11);
            }

            if(bandType == 2)
            {
                IplImage* bmps = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
                cvSmooth(gimg, bmps, CV_GAUSSIAN, 11);
                for(int y = 0; y < h; y++)
                {
                    for(int x = 0; x < w; x++)
                    {
                        CvScalar s1, s2;
                        s1 = cvGet2D(gimg, y, x);
                        s2 = cvGet2D(bmps, y, x);
                        s1.val[0] = s1.val[0] - s2.val[0];
                        cvSet2D(gimg, y, x, s1);
                    }
                }
                cvReleaseImage(&bmps);
            }

            for(int y = 0; y < h; y++)
            {
                for(int x = 0; x < w; x++)
                {
                    CvScalar s;
                    s = cvGet2D(gimg, y, x);
                    if(transpose)
                    {
                        imgArr[y * w + x] = s.val[0];
                    }
                    else
                    {
                        imgArr[x * h + y] = s.val[0];
                    }
                }
            }

            cvReleaseImage(&gimg);
        }
        else
        {
            printf("!width, height \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        printf("WARNING file %s does not exists!\n", fileNameOrigStr.c_str());
        exit(EXIT_FAILURE);
    }
}

void saveSeedsToFile(staticVector<seedPoint>* seeds, const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size = seeds->size();
        fwrite(&size, sizeof(int), 1, f);
        for(int i = 0; i < size; i++)
        {
            seedPoint* sp = &(*seeds)[i];
            seed_io_block sb;
            sb.area = sp->area;
            sb.ncams = sp->cams.size();
            sb.op = sp->op;
            sb.pixSize = sp->pixSize;
            sb.segId = sp->segId;
            sb.xax = sp->xax;
            sb.yax = sp->yax;

            fwrite(&sb, sizeof(seed_io_block), 1, f);

            for(int j = 0; j < sb.ncams; j++)
            {
                unsigned short c = sp->cams[j];
                fwrite(&c, sizeof(unsigned short), 1, f);
            }

            for(int j = 0; j < sb.ncams + 1; j++)
            {
                point2d sh = sp->cams.shifts[j];
                fwrite(&sh, sizeof(point2d), 1, f);
            }
        }
        fclose(f);
    }
}

void saveSeedsToFile(staticVector<seedPoint>* seeds, int refImgFileId, multiviewInputParams* mip, int mv_file_type)
{
    std::string fileName = mv_getFileName(mip, refImgFileId, mv_file_type);
    saveSeedsToFile(seeds, fileName);
}

bool loadSeedsFromFile(staticVector<seedPoint>** seeds, const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == nullptr)
    {
        // printf("file does not exists \n");
        *seeds = new staticVector<seedPoint>(1);
        return false;
    }

    int size;
    fread(&size, sizeof(int), 1, f);
    *seeds = new staticVector<seedPoint>(std::max(1, size));

    for(int i = 0; i < size; i++)
    {
        seedPoint sp = seedPoint();
        seed_io_block sb;

        fread(&sb, sizeof(seed_io_block), 1, f);

        sp.area = sb.area;
        sp.cams.reserve(sb.ncams);
        sp.op = sb.op;

        // printf("seed rx: %i, ry: %i \n", (int)sp->op.n.rx, (int)sp->op.n.ry);

        sp.pixSize = sb.pixSize;
        sp.segId = sb.segId;
        sp.xax = sb.xax;
        sp.yax = sb.yax;

        for(int j = 0; j < sb.ncams; j++)
        {
            unsigned short c;
            fread(&c, sizeof(unsigned short), 1, f);
            sp.cams.push_back(c);
        }

        for(int j = 0; j < sb.ncams + 1; j++)
        {
            point2d sh;
            fread(&sh, sizeof(point2d), 1, f);
            sp.cams.shifts[j] = sh;
        }

        (*seeds)->push_back(sp);
    }

    fclose(f);
    return true;
}

bool loadSeedsFromFile(staticVector<seedPoint>** seeds, int refImgFileId, multiviewInputParams* mip, int mv_file_type)
{
    std::string fileName = mv_getFileName(mip, refImgFileId, mv_file_type);
    return loadSeedsFromFile(seeds, fileName);
}

int getSeedsSizeFromFile(int refImgFileId, multiviewInputParams* mip, int mv_file_type)
{
    FILE* f = mv_openFile(mip, refImgFileId, mv_file_type, "rb");
    if(f == nullptr)
    {
        return 0;
    }

    int size;
    fread(&size, sizeof(int), 1, f);
    fclose(f);
    return size;
}

int getGrowedSizeFromFile(int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_growed, "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size;
        fread(&size, sizeof(int), 1, f);
        fclose(f);
        return size;
    }

    return 0;
}


void saveUniqueIdAliveToFile(std::vector<bool>* uniqueIdAlive, multiviewInputParams* mip)
{
    std::string fname = mip->mvDir + mip->prefix + "_alive.txt";
    FILE* f = fopen(fname.c_str(), "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int32_t sizei = (int32_t)uniqueIdAlive->size();
        fwrite(&sizei, sizeof(int32_t), 1, f);

        for(int i = 0; i < sizei; i++)
        {
            bool b = (*uniqueIdAlive)[i];
            fwrite(&b, sizeof(bool), 1, f);
        }
        fclose(f);
    }
}

void loadUniqueIdAliveFromFile(std::vector<bool>* uniqueIdAlive, multiviewInputParams* mip)
{
    std::string fname = mip->mvDir + mip->prefix + "_alive.txt";
    FILE* f = fopen(fname.c_str(), "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int32_t size;
        fread(&size, sizeof(int32_t), 1, f);
        uniqueIdAlive->reserve(size);
        for(int i = 0; i < size; i++)
        {
            bool b;
            fread(&b, sizeof(bool), 1, f);
            uniqueIdAlive->push_back(b);
        }
        fclose(f);
    }
}

void deletePremtachFiles(multiviewInputParams mip, int ncams)
{
    // delete files
    for(int rc = 0; rc < ncams; rc++)
    {
        for(int tc = 0; tc < ncams; tc++)
        {
            std::string fileName = mv_getFileNamePrefixRcTc(&mip, rc + 1, tc + 1);
            fileName += "_prematched.txt";
            remove(fileName.c_str());
        }
    }
}
void deleteFilesOfType(multiviewInputParams& mip, int ncams, int mv_file_type)
{
    // delete files
    long t1 = initEstimate();
    for(int rc = 0; rc < ncams; rc++)
    {
        std::string fileName = mv_getFileName(&mip, rc + 1, mv_file_type);
        remove(fileName.c_str());
        printfEstimate(rc, ncams, t1);
    }
    finishEstimate();
}

void saveOrientedPointsToFile(staticVector<orientedPoint>* ops, int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_op, "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size = ops->size();
        fwrite(&size, sizeof(int), 1, f);
        fwrite(&(*ops)[0], sizeof(orientedPoint), size, f);
        fclose(f);
    }
}

staticVector<orientedPoint>* loadOrientedPointsFromFile(int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_op, "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size;
        fread(&size, sizeof(int), 1, f);
        staticVector<orientedPoint>* ops = new staticVector<orientedPoint>(size);
        ops->resize(size);
        fread(&(*ops)[0], sizeof(orientedPoint), size, f);
        fclose(f);

        return ops;
    }

    return nullptr;
}

int getNumOrientedPointsFromFile(int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_op, "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size;
        fread(&size, sizeof(int), 1, f);
        fclose(f);

        return size;
    }

    return 0;
}

/*
void loadVisibilityMapFromFileWithAllocation(bool **vis, int refImgFileId, multiviewInputParams *mip)
{
        FILE *f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_visibility_map, "rb");
        if (f == NULL)
        {
                //
        }else
        {
                int size;
                fread(&size, sizeof(int), 1, f);
                *vis = new bool[size];
                fread(&((*vis)[0]), sizeof(bool), size, f);
                fclose(f);
        };
}

void saveVisibilityMapToFile(bool *vis, int size, int refImgFileId, multiviewInputParams *mip)
{
        FILE *f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_visibility_map, "wb");
        if (f == NULL)
        {
                //
        }else
        {
                fwrite(&size, sizeof(int), 1, f);
                fwrite(&vis[0], sizeof(bool), size, f);
                fclose(f);
        };
}
*/

bool loadPairConfidenceMatrixFromFileWithAllocation(unsigned char** cm, multiviewInputParams* mip,
                                                    const std::string& name)
{
    std::string fname = mip->mvDir + mip->prefix + name;
    FILE* f = fopen(fname.c_str(), "rb");
    if(f == nullptr)
    {
        *cm = nullptr;
        return false;
    }

    int32_t size;
    fread(&size, sizeof(int32_t), 1, f);
    *cm = new unsigned char[size];
    fread(&((*cm)[0]), sizeof(unsigned char), size, f);
    fclose(f);
    return true;
}

void savePairConfidenceMatrixToFile(unsigned char* cm, int32_t size, multiviewInputParams* mip, const std::string& name)
{
    std::string fname = mip->mvDir + mip->prefix + name;
    FILE* f = fopen(fname.c_str(), "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        fwrite(&size, sizeof(int32_t), 1, f);
        fwrite(&cm[0], sizeof(unsigned char), size, f);
        fclose(f);
    }
}

void savePrematchedToFile(outStruct* outv, int size, int refImgFileId, int tarImgFileId, multiviewInputParams* mip)
{
    std::string fileName = mv_getFileNamePrefixRcTc(mip, refImgFileId, tarImgFileId);
    fileName += "_prematched.txt";
    FILE* f = fopen(fileName.c_str(), "wb");

    if(f == nullptr)
    {
        //
    }
    else
    {
        fwrite(&size, sizeof(int), 1, f);
        fwrite(&outv[0], sizeof(outStruct), size, f);
        fclose(f);
    }
}

outStruct* loadPrematchedFromFile(int* sz, int refImgFileId, int tarImgFileId, multiviewInputParams* mip)
{
    outStruct* outv = nullptr;

    std::string fileName = mv_getFileNamePrefixRcTc(mip, refImgFileId, tarImgFileId);
    fileName += "_prematched.txt";
    FILE* f = fopen(fileName.c_str(), "rb");

    if(f == nullptr)
    {
        *sz = 0;
        return nullptr;
    }

    int size;
    fread(&size, sizeof(int), 1, f);
    outv = new outStruct[size];
    fread(&outv[0], sizeof(outStruct), size, f);
    fclose(f);

    *sz = size;

    return outv;
}
/*
bool loadAgreedVisMapToFileFromFileWithoutAllocation(idValue* vm, int refImgFileId, multiviewInputParams* mip)
{
    int32_t size = (mip->getWidth(rc)) * (mip->getHeight(rc));
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_agreedVisMap, "rb");
    if(f == nullptr)
    {
        for(int i = 0; i < size; i++)
        {
            vm[i] = idValue(-1, -1.0);
        }
        return false;
    }

    fwrite(&vm[0], sizeof(idValue), size, f);
    fclose(f);
    return true;
}

bool saveAgreedVisMapToFile(idValue* vm, int refImgFileId, multiviewInputParams* mip)
{
    int32_t size = (mip->getWidth(rc)) * (mip->getHeigh(rc));
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_agreedVisMap, "wb");
    if(f == nullptr)
    {
        return false;
    }

    fwrite(&vm[0], sizeof(idValue), size, f);
    fclose(f);
    return true;
}

bool loadDiskSizeMapFromFileWithoutAllocation(int* vm, int refImgFileId, multiviewInputParams* mip)
{
    int32_t size = (mip->getWidth(rc)) * (mip->getHeight(rc));
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_diskSizeMap, "rb");
    if(f == nullptr)
    {
        for(int i = 0; i < size; i++)
        {
            vm[i] = (int)0;
        }
        return false;
    }

    fread(&vm[0], sizeof(int), size, f);
    fclose(f);
    return true;
}

bool saveDiskSizeMapToFile(int* vm, int refImgFileId, multiviewInputParams* mip)
{
    int32_t size = (mip->getWidth(rc)) * (mip->getHeight(rc));
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_diskSizeMap, "wb");
    if(f == nullptr)
    {
        return false;
    }

    fwrite(&vm[0], sizeof(int), size, f);
    fclose(f);
    return true;
}
*/
void deleteAllFiles(multiviewInputParams* mip)
{
    printf("deleteing temporary files\n");
    // deletePremtachFiles(*mip,mip->getNbCameras());
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_seeds);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_growed);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_nearMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_occMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_op);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_seeds_flt);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_seeds_prm);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_seeds_sfm);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_refinedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_growedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_graphCutPts);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_graphCutMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_graphCutMesh);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_agreedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_agreedPts);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_agreedMesh);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_nearMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_occMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_op);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_nearestAgreedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_segPlanes);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_agreedVisMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_diskSizeMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_depthMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), mip->MV_FILE_TYPE_simMap);
}

void saveToImg(const std::string& fileName, unsigned char* img, int width, int height, bool T)
{
    IplImage* bmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            unsigned char g;
            if(T)
            {
                g = img[y * width + x];
            }
            else
            {
                g = img[x * height + y];
            }

            CvScalar s;
            s.val[0] = g;
            cvSet2D(bmp, y, x, s);
        }
    }

    if(cvSaveImage(fileName.c_str(), bmp) == 0)
        printf("Could not save: %s\n", fileName.c_str());

    cvReleaseImage(&bmp);
}

void saveToImgResize(const std::string& fileName, unsigned char* img, int width, int height, int widthNew,
                     int heightNew, bool T)
{
    IplImage* bmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            unsigned char g;
            if(T)
            {
                g = img[y * width + x];
            }
            else
            {
                g = img[x * height + y];
            }

            CvScalar s;
            s.val[0] = g;
            cvSet2D(bmp, y, x, s);
        }
    }

    IplImage* bmpr = cvCreateImage(cvSize(widthNew, heightNew), IPL_DEPTH_8U, 1);
    cvResize(bmp, bmpr);
    if(cvSaveImage(fileName.c_str(), bmpr) == 0)
        printf("Could not save: %s\n", fileName.c_str());

    cvReleaseImage(&bmp);
    cvReleaseImage(&bmpr);
}

unsigned char* imgResize(unsigned char* img, int width, int height, int widthNew, int heightNew, bool T)
{
    IplImage* bmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            unsigned char g;
            if(T)
            {
                g = img[y * width + x];
            }
            else
            {
                g = img[x * height + y];
            }

            CvScalar s;
            s.val[0] = g;
            cvSet2D(bmp, y, x, s);
        }
    }

    IplImage* bmpr = cvCreateImage(cvSize(widthNew, heightNew), IPL_DEPTH_8U, 1);
    cvResize(bmp, bmpr);

    unsigned char* imgo = new unsigned char[widthNew * heightNew];

    for(int x = 0; x < widthNew; x++)
    {
        for(int y = 0; y < heightNew; y++)
        {
            CvScalar s;
            s = cvGet2D(bmp, y, x);
            unsigned char g = s.val[0];
            if(T)
            {
                imgo[y * widthNew + x] = g;
            }
            else
            {
                imgo[x * heightNew + y] = g;
            }
        }
    }

    cvReleaseImage(&bmp);
    cvReleaseImage(&bmpr);

    return imgo;
}

void saveToImg(const std::string& fileName, Color* img, int width, int height, bool T)
{
    IplImage* bmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            Color g;
            if(T)
            {
                g = img[y * width + x];
            }
            else
            {
                g = img[x * height + y];
            }

            CvScalar s;
            s.val[0] = (unsigned char)std::min(255.0f, std::max(0.0f, g.b));
            s.val[1] = (unsigned char)std::min(255.0f, std::max(0.0f, g.g));
            s.val[2] = (unsigned char)std::min(255.0f, std::max(0.0f, g.r));
            cvSet2D(bmp, y, x, s);
        }
    }

    if(cvSaveImage(fileName.c_str(), bmp) == 0)
        printf("Could not save: %s\n", fileName.c_str());

    cvReleaseImage(&bmp);
}

bool getDepthMapInfo(int refImgFileId, multiviewInputParams* mip, float& mindepth, float& maxdepth,
                     staticVector<int>** tcams)
{
    *tcams = nullptr;
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_depthMapInfo, "r");
    if(f == nullptr)
    {
        printf("WARNING!!!! depth map info %i does not exists!\n", refImgFileId);
        return false;
    }

    int ntcams;
    fscanf(f, "minDepth %f, maxDepth %f, ntcams %i, tcams", &mindepth, &maxdepth, &ntcams);
    (*tcams) = new staticVector<int>(ntcams);
    for(int c = 0; c < ntcams; c++)
    {
        int tc;
        fscanf(f, " %i", &tc);
        (*tcams)->push_back(tc);
    }
    fclose(f);
    return true;
}

bool getDepthMapInfoDepthLimits(int refImgFileId, multiviewInputParams* mip, float& mindepth, float& maxdepth)
{
    FILE* f = mv_openFile(mip, refImgFileId, mip->MV_FILE_TYPE_depthMapInfo, "r");
    if(f == nullptr)
    {
        printf("WARNING!!!!\n");
        return false;
    }

    int ntcams;
    fscanf(f, "minDepth %f, maxDepth %f, ntcams %i, tcams", &mindepth, &maxdepth, &ntcams);
    fclose(f);
    return true;
}

void saveSegToImgCol(rgb* colors, const std::string& fileName, int* img, int width, int height, bool T)
{
    IplImage* bmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            int g;
            if(T)
            {
                g = img[y * width + x];
            }
            else
            {
                g = img[x * height + y];
            }

            CvScalar s;

            if(g < 0)
            {
                s.val[0] = 0;
                s.val[1] = 0;
                s.val[2] = 0;
            }
            else
            {
                rgb b = colors[g];
                s.val[0] = b.b;
                s.val[1] = b.g;
                s.val[2] = b.r;
            }
            cvSet2D(bmp, y, x, s);
        }
    }

    if(cvSaveImage(fileName.c_str(), bmp) == 0)
        printf("Could not save: %s\n", fileName.c_str());

    cvReleaseImage(&bmp);
}
void saveSegToImg(const std::string& fileName, int* img, int width, int height, bool T)
{
    int maxid = 0;
    for(int i = 0; i < width * height; i++)
        maxid = std::max(img[i], maxid);

    // pick random colors for each component
    rgb* colors = new rgb[maxid + 1];
    for(int i = 0; i < maxid + 1; i++)
    {
        colors[i] = rgb_random();
    }

    saveSegToImgCol(colors, fileName, img, width, height, T);

    delete[] colors;
}

void saveImgAsPgm(const std::string& img, const std::string& imgpgm)
{
    IplImage* bmp = cvLoadImage(img.c_str());
    IplImage* gimg = cvCreateImage(cvSize(bmp->width, bmp->height), IPL_DEPTH_8U, 1);
    cvCvtColor(bmp, gimg, CV_BGR2GRAY);
    if(cvSaveImage(imgpgm.c_str(), gimg) == 0)
        printf("Could not save: %s\n", imgpgm.c_str());
    cvReleaseImage(&gimg);
    cvReleaseImage(&bmp);
}

bool IsDots(const char* str)
{
    std::string dd = ".";
    std::string ddd = ".";
    return !((strcmp(str, dd.c_str()) != 0) && (strcmp(str, ddd.c_str()) != 0));
}

bool DeleteDirectory(const std::string& sPath)
{
    boost::filesystem::remove_all(sPath);
    return true;
}

bool getDirectoryFiles(std::vector<std::string>& out, const std::string& sPath, const std::string& ext)
{
    namespace fs = boost::filesystem;

    fs::path apk_path(sPath);
    fs::directory_iterator end;

    for(fs::directory_iterator i(apk_path); i != end; ++i)
    {
        if(ext.empty() || i->path().extension() == ext)
            out.push_back(i->path().string());
    }

    return true;
}

void imagesc(const std::string& imageFileName, unsigned short* data, int w, int h, unsigned short minVal,
             unsigned short maxVal, bool T)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {

            float val;
            if(!T)
            {
                val = (float)data[x * h + y];
            }
            else
            {
                val = (float)data[y * w + x];
            }
            float s = 1.0f - ((float)maxVal - std::max((float)minVal, (float)val)) / ((float)maxVal - (float)minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    if(cvSaveImage(imageFileName.c_str(), img) == 0)
        printf("Could not save: %s\n", imageFileName.c_str());
    cvReleaseImage(&img);
}

void imagesc(const std::string& imageFileName, float* data, int w, int h, float minVal, float maxVal, bool T)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {

            float val;
            if(!T)
            {
                val = data[x * h + y];
            }
            else
            {
                val = data[y * w + x];
            }
            float s = 1.0f - (maxVal - std::max(minVal, val)) / (maxVal - minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    if(cvSaveImage(imageFileName.c_str(), img) == 0)
        printf("Could not save: %s\n", imageFileName.c_str());
    cvReleaseImage(&img);
}

void imagesc(const std::string& imageFileName, int* data, int w, int h, int minVal, int maxVal, bool T)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {

            float val;
            if(!T)
            {
                val = (float)data[x * h + y];
            }
            else
            {
                val = (float)data[y * w + x];
            }
            float s = 1.0f - ((float)maxVal - std::max((float)minVal, val)) / ((float)maxVal - (float)minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    if(cvSaveImage(imageFileName.c_str(), img) == 0)
        printf("Could not save: %s\n", imageFileName.c_str());
    cvReleaseImage(&img);
}

void imagesc16bitGrayNpg(const std::string& imageFileName, float* data, int w, int h, float minVal, float maxVal)
{
    // IplImage* img = cvCreateImage(cvSize(w,h), IPL_DEPTH_16U, 1);
    cv::Mat mat_img(h, w, CV_16U);

    for(int y = 0; y < h; y++)
    {
        short int* My = mat_img.ptr<short int>(y);
        for(int x = 0; x < w; x++)
        {
            float val = data[y * w + x];
            float s = 1.0f - (maxVal - std::max(minVal, val)) / (maxVal - minVal);
            short int ss = (short int)(s * 65535.0f);
            /*
            CvScalar c;
            c.val[0] = ss;
            c.val[1] = ss;
            c.val[2] = ss;
            cvSet2D(img,y,x,c);
            */
            My[x] = ss;
        }
    }

    //	cv::Mat mat_img(img, true);
    cv::imwrite(imageFileName, mat_img);

    // if(!cvSaveImage(imageFileName.c_str(),img)) printf("Could not save: %s\n",imageFileName.c_str());
    // cvReleaseImage(&img);
}

void imagesc(const std::string& imageFileName, float* data, int w, int h, bool T)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {

            float val;
            if(!T)
            {
                val = data[x * h + y];
            }
            else
            {
                val = data[y * w + x];
            }
            CvScalar c;
            c.val[0] = val;
            c.val[1] = val;
            c.val[2] = val;
            cvSet2D(img, y, x, c);
        }
    }

    if(cvSaveImage(imageFileName.c_str(), img) == 0)
        printf("Could not save: %s\n", imageFileName.c_str());
    cvReleaseImage(&img);
}

void readSifts(const std::string& fileName, staticVector<float>** descriptors, staticVector<SiftKeypoint>** keys)
{
    // std::string fname = "D:/jancom1/DATA/templeRing/planeSweepingScale1/00001.ppm.sift";

    FILE* f = fopen(fileName.c_str(), "rb");

    int n;
    fread(&n, sizeof(int), 1, f);

    // printf("%i\n",n);
    // float mbsifts = ((float)(n*sizeof(SiftGPU::SiftKeypoint)+n*128*sizeof(float))/1024.0)/1024.0;
    // printf("%i sifts contains %f MB \n",n,mbsifts);

    staticVector<SiftKeypoint>* Keys = new staticVector<SiftKeypoint>(n);
    Keys->resize(n);
    staticVector<float>* Descriptors = new staticVector<float>(128 * n);
    Descriptors->resize(128 * n);

    fread(&(*Keys)[0], sizeof(SiftKeypoint), n, f);
    fread(&(*Descriptors)[0], sizeof(float), 128 * n, f);

    fclose(f);

    (*keys) = Keys;
    (*descriptors) = Descriptors;
}

void splitString(const std::string& str, const std::string& delimiters, std::vector<std::string>& tokens)
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while(std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}
