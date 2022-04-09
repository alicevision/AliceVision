#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include "photometricDataIO.hpp"
#include "photometricStereo.hpp"

namespace fs = boost::filesystem;

void photometricStereo(const std::string& inputPath, const std::string& dataFolderPath, const std::string& outputPath, const size_t HS_order, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo)
{
    size_t dim = 3;
    if(HS_order == 2)
    {
        dim = 9;
    }

    std::vector<std::string> imageList;
    std::string pictureFolder = inputPath + "/PS_Pictures/";
    getPicturesNames(pictureFolder, imageList);

    Eigen::MatrixXf convertionMatrix = Eigen::MatrixXf::Zero(3,3); // Convertion matrix

    std::vector<std::array<float, 3>> intList; // Light intensities

    Eigen::MatrixXf lightMat(imageList.size(), dim); //Light directions

    aliceVision::image::Image<float> mask;
    loadPSData(dataFolderPath, HS_order, intList, lightMat, convertionMatrix, mask);

    photometricStereo(imageList, intList, lightMat, mask, normals, albedo);

    writePSResults(outputPath, normals, albedo);

}

void photometricStereo(const aliceVision::sfmData::SfMData& sfmData, const std::string& dataFolderPath, const std::string& outputPath, const size_t HS_order, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo)
{
    size_t dim = 3;
    if(HS_order == 2)
    {
        dim = 9;
    }

    std::vector<std::string> imageList;

    std::map<aliceVision::IndexT, std::vector<aliceVision::IndexT>> viewsPerPoseId;

    for(auto& viewIt: sfmData.getViews())
    {
        viewsPerPoseId[viewIt.second->getPoseId()].push_back(viewIt.second->getViewId());
    }

    Eigen::MatrixXf convertionMatrix = Eigen::Matrix<float, 3, 3>::Identity(); // Convertion matrix
    std::vector<std::array<float, 3>> intList; // Light intensities
    aliceVision::image::Image<float> mask;

    for(auto& posesIt: viewsPerPoseId)
    {
        ALICEVISION_LOG_INFO("Pose Id: " << posesIt.first);
        std::vector<aliceVision::IndexT>& viewIds = posesIt.second;
        for(auto& viewId: viewIds)
        {
            const fs::path imagePath = fs::path(sfmData.getView(viewId).getImagePath());
            if(!boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
            {
                ALICEVISION_LOG_INFO("  - " << imagePath.string());
                imageList.push_back(imagePath.string());
            }
        }

        Eigen::MatrixXf lightMat(imageList.size(), dim); //Light directions

        loadPSData(dataFolderPath, HS_order, intList, lightMat, convertionMatrix, mask);
        photometricStereo(imageList, intList, lightMat, mask, normals, albedo);
        writePSResults(outputPath, normals, albedo, posesIt.first);
    }

}

void photometricStereo(const std::vector<std::string>& imageList, const std::vector<std::array<float, 3>>& intList, const Eigen::MatrixXf& lightMat, const aliceVision::image::Image<float>& mask, aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<aliceVision::image::RGBfColor>& albedo)
{
    std::vector<int> indexes;
    getIndMask(mask, indexes);
    size_t maskSize = indexes.size();

    const int pictRows = mask.rows();
    const int pictCols = mask.cols();

    Eigen::MatrixXf currentPicture(3,pictRows*pictCols);
    Eigen::MatrixXf allPictures(3*imageList.size(), pictRows*pictCols);
    Eigen::MatrixXf imMat(3*imageList.size(), maskSize);

    std::string picturePath;
    std::string pictureName;

    // Read pictures :
    for (size_t i = 0; i < imageList.size(); ++i)
    {
        picturePath = imageList.at(i);

        aliceVision::image::Image<aliceVision::image::RGBfColor> imageFloat;
        aliceVision::image::ImageReadOptions options;
        options.outputColorSpace = aliceVision::image::EImageColorSpace::NO_CONVERSION;
        aliceVision::image::readImage(picturePath, imageFloat, options);
        intensityScaling(intList.at(i), imageFloat);
        image2PsMatrix(imageFloat, currentPicture);

        allPictures.block(3*i,0,3,allPictures.cols()) << currentPicture;
    }
    applyMask(allPictures, indexes, imMat);

    Eigen::MatrixXf normalsVect = Eigen::MatrixXf::Zero(lightMat.cols(),pictRows*pictCols);
    Eigen::MatrixXf albedoVect = Eigen::MatrixXf::Zero(3,pictRows*pictCols);

    // Pixelwise normal and albedo evaluation :
    for (size_t i = 0; i < maskSize; ++i)
    {
        // Create I matrix for current pixel :
        Eigen::MatrixXf allPixelValues = imMat.col(i);

        Eigen::Map<Eigen::MatrixXf> IPixel_transposed(allPixelValues.data(), 3, imageList.size());
        Eigen::MatrixXf IPixel(imageList.size(),3);
        IPixel = IPixel_transposed.transpose();

        // M_0 = pinv(S)*I :
        Eigen::MatrixXf M_0 = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(IPixel);

        // SVD(M_0) :
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(M_0, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::MatrixXf U = svd.matrixU();
        Eigen::MatrixXf V = svd.matrixV();
        Eigen::MatrixXf n_pixel = U.col(0);
        Eigen::MatrixXf rho_pixel = V.col(0);

        if (n_pixel(2) > 0)
        {
            n_pixel = -n_pixel;
            rho_pixel = -rho_pixel;
        }

        int currentIdx = indexes.at(i); // index in picture
        normalsVect.block(0,currentIdx, normalsVect.rows(), 1) = n_pixel;

        albedoVect.block(0,currentIdx, 3, 1) = rho_pixel;
    }

    aliceVision::image::Image<aliceVision::image::RGBfColor> normalsIm(pictCols,pictRows);
    normals2picture(normalsVect, normalsIm);
    normals = normalsIm;

    aliceVision::image::Image<aliceVision::image::RGBfColor> albedoIm(pictCols,pictRows);
    normals2picture(albedoVect, albedoIm);
    albedo = albedoIm;
}

void loadPSData(const std::string& folderPath, const size_t& HS_order, std::vector<std::array<float, 3>>& intList, Eigen::MatrixXf& lightMat, Eigen::MatrixXf& convertionMatrix, aliceVision::image::Image<float>& mask)
{
    std::string intFileName;
    std::string pathToCM;
    std::string dirFileName;
    std::string maskName;

    // Light instensities :
    intFileName = folderPath + "/light_intensities.txt";
    loadLightIntensities(intFileName, intList);

    // Convertion matrix :
    pathToCM = folderPath + "/convertionMatrix.txt";
    readMatrix(pathToCM, convertionMatrix);

    // Light directions :
    if(HS_order == 0)
    {
        dirFileName = folderPath + "/light_directions.txt";
        loadLightDirections(dirFileName, convertionMatrix, lightMat);
    } else if (HS_order == 2) {
        dirFileName = folderPath + "/light_directions_HS.txt";
        loadLightHS(dirFileName, lightMat);
    }
    
    // Mask :
    maskName = folderPath + "/mask.png";
    loadMask(maskName, mask);
}

void getPicturesNames(const std::string& folderPath, std::vector<std::string>& imageList)
{
    boost::filesystem::directory_iterator endItr;
    for(boost::filesystem::directory_iterator itr(folderPath); itr != endItr; ++itr)
    {
      std::string currentFilePath = itr->path().string();
      
      std::string fileExtension = boost::filesystem::extension(currentFilePath);
      std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(), ::tolower);

      if(fileExtension == ".png")
      {
        imageList.push_back(currentFilePath);
      }
    }

    std::sort(imageList.begin(),imageList.end(),compareFunction);//sort the vector
}

bool compareFunction (std::string a, std::string b) {return a<b;}
