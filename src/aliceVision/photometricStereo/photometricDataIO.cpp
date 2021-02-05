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
