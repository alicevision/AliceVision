#include <aliceVision/image/io.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include "photometricDataIO.hpp"

void loadLightIntensities(const std::string& intFileName, std::vector<std::array<float, 3>>& intList)
{
    std::stringstream stream;
    std::string line;
    float x,y,z;

    std::fstream intFile;
    intFile.open(intFileName, std::ios::in);

    if (intFile.is_open())
    {

        while(!intFile.eof())
        {
            std::getline(intFile,line);
            stream.clear();
            stream.str(line);

            stream >> x >> y >> z;
            std::array<float, 3> v = {x, y, z};
            intList.push_back(v);
        }
        intList.pop_back();
        intFile.close();
    }
}


void loadLightDirections(const std::string& dirFileName, const Eigen::MatrixXf& convertionMatrix, Eigen::MatrixXf& lightMat)
{
    std::stringstream stream;
    std::string line;
    float x,y,z;

    std::fstream dirFile;
    dirFile.open(dirFileName, std::ios::in);

    if (dirFile.is_open())
    {
        int lineNumber = 0;

        while(!dirFile.eof())
        {
            getline(dirFile,line);
            stream.clear();
            stream.str(line);

            stream >> x >> y >> z;
            for(int i = 0; i< 3; ++i)
            {
                if(lineNumber < lightMat.rows())
                {
                  lightMat(lineNumber, 0) = convertionMatrix(0,0)*x + convertionMatrix(0,1)*y + convertionMatrix(0,2)*z;
                  lightMat(lineNumber, 1) = convertionMatrix(1,0)*x + convertionMatrix(1,1)*y + convertionMatrix(1,2)*z;
                  lightMat(lineNumber, 2) = convertionMatrix(2,0)*x + convertionMatrix(2,1)*y + convertionMatrix(2,2)*z;
                  ++lineNumber;
                }
            }
        }
        dirFile.close();
    }
}

void loadMask(std::string const& maskName, aliceVision::image::Image<float>& mask)
{
    aliceVision::image::ImageReadOptions options;
    options.outputColorSpace = aliceVision::image::EImageColorSpace::SRGB;
    aliceVision::image::readImage(maskName, mask, options);
}

void getIndMask(aliceVision::image::Image<float> const& mask, std::vector<int>& indexes)
{
    const int nbRows = mask.rows();
    const int nbCols = mask.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if(mask(i,j) != 0)
            {
                int currentIndex = j*nbRows + i;
                indexes.push_back(currentIndex);
            }
        }
    }
}

void intensityScaling(std::array<float, 3> const& intensities, aliceVision::image::Image<aliceVision::image::RGBfColor>& imageToScale)
{
    int nbRows = imageToScale.rows();
    int nbCols = imageToScale.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            for(int ch = 0; ch < 3; ++ch)
            {
                imageToScale(i,j)(ch) /= intensities[ch];
            }
        }
    }
}

void image2PsMatrix(const aliceVision::image::Image<aliceVision::image::RGBfColor>& imageIn, Eigen::MatrixXf& imageOut)
{
    int nbRows = imageIn.rows();
    int nbCols = imageIn.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            int index = j*nbRows + i;
            for(int ch = 0; ch < 3; ++ch)
            {
                imageOut(ch, index) = imageIn(i,j)(ch);
            }
        }
    }
}

void applyMask(const Eigen::MatrixXf& inputMatrix, const std::vector<int>& maskIndexes, Eigen::MatrixXf& maskedMatrix)
{
    for (int j = 0; j < maskedMatrix.cols(); ++j)
    {
        int indexInMask = maskIndexes.at(j);
        for (int i = 0; i < maskedMatrix.rows(); ++i)
        {
            maskedMatrix(i,j) = inputMatrix(i, indexInMask);
        }
    }
}

void normals2picture(const Eigen::MatrixXf& normalsMatrix, aliceVision::image::Image<aliceVision::image::RGBfColor>& normalsIm)
{
    int nbRows = normalsIm.rows();
    int nbCols = normalsIm.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            int currentInd = j*nbRows + i;
            for (int ch = 0; ch < 3; ++ch)
            {
                normalsIm(i,j)(ch) = normalsMatrix(ch,currentInd);
            }
        }
    }
}

void convertNormalMap2png(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normalsIm, aliceVision::image::Image<aliceVision::image::RGBColor>& normalsImPNG)
{
    int nbRows = normalsIm.rows();
    int nbCols = normalsIm.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            for (int ch = 0; ch < 3; ++ch)
            {
                normalsImPNG(i,j)(ch) = floor((normalsIm(i,j)(ch) + 1)*127.5);
            }
        }
    }
}

void readMatrix(const std::string& fileName, Eigen::MatrixXf& matrix)
{
    int nbRows = matrix.rows();
    int nbCols = matrix.cols();

    std::stringstream stream;
    std::string line;

    std::fstream matFile;
    matFile.open(fileName, std::ios::in);

    if (matFile.is_open())
    {
    for (int row = 0; row < nbRows; row++)
        for (int col = 0; col < nbCols; col++)
        {
            float item = 0.0;
            matFile >> item;
            matrix(row, col) = item;
        }
    }
    matFile.close();
}
