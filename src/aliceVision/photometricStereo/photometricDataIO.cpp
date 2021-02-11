#include <aliceVision/image/io.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include "photometricDataIO.hpp"

void loadLightIntensities(const std::string& intFileName, const std::vector<int>& usedPictures, std::vector<std::array<float, 3>>& intList)
{
    std::stringstream stream;
    std::string line;
    float x,y,z;
    
    std::fstream intFile;
    intFile.open(intFileName, std::ios::in);
    
    if (intFile.is_open())
    {
		int cpt = 1;
		
        while(!intFile.eof())
        {
			std::getline(intFile,line);
			stream.clear();
			stream.str(line);
				
			if(std::find(usedPictures.begin(), usedPictures.end(), cpt) != usedPictures.end())
			{
				stream >> x >> y >> z;
				std::array<float, 3> v = {x, y, z};
				intList.push_back(v);
			}
			
			++cpt;
        }
        intFile.close();
    }
}

void loadLightDirections(const std::string& dirFileName, const std::vector<int>& usedPictures, Eigen::MatrixXf& lightMat)
{
    std::stringstream stream;
    std::string line;
    float x,y,z;
    
    std::fstream dirFile;
    dirFile.open(dirFileName, std::ios::in);
    
    if (dirFile.is_open())
    {
        int cpt = 1;
		int lineNumber = 0;
		
        while(! dirFile.eof())
        {
            getline(dirFile,line);
            stream.clear();
            stream.str(line);
		
			if(std::find(usedPictures.begin(), usedPictures.end(), cpt) != usedPictures.end())
			{
				stream >> x >> y >> z;
                for(int i = 0; i< 3; ++i)
                {
                    lightMat(lineNumber, 0) = x;
                    lightMat(lineNumber, 1) = y;
                    lightMat(lineNumber, 2) = z;
				
                    ++lineNumber;

                }
			}
			++cpt;
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
