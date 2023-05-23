#include "photometricStereo.hpp"
#include "photometricDataIO.hpp"

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/resampling.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace photometricStereo {

void photometricStereo(const std::string& inputPath, const std::string& lightData, const std::string& outputPath, const PhotometricSteroParameters& PSParameters, image::Image<image::RGBfColor>& normals, image::Image<image::RGBfColor>& albedo)
{

    size_t dim = 3;
    if(PSParameters.SHOrder == 2)
    {
        dim = 9;
    }

    std::vector<std::string> imageList;
    std::string pictureFolder = inputPath + "/PS_Pictures/";
    getPicturesNames(pictureFolder, imageList);

    std::vector<std::array<float, 3>> intList; // Light intensities
    Eigen::MatrixXf lightMat(imageList.size(), dim); //Light directions

    if(fs::is_directory(lightData))
    {
        loadPSData(lightData, PSParameters.SHOrder, intList, lightMat);
    }
    else
    {
        buildLigtMatFromJSON(lightData, imageList, lightMat, intList);
    }

    image::Image<float> mask;
    fs::path lightDataPath = fs::path(lightData);
    std::string maskName = lightDataPath.remove_filename().string() + "/mask.png";
    loadMask(maskName, mask);

    std::string pathToAmbiant = "";

    if(PSParameters.removeAmbiant)
    {
        for (auto& currentPath : imageList)
        {
            const fs::path imagePath = fs::path(currentPath);
            if(boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
            {
                pathToAmbiant = imagePath.string();
            }
        }
    }

    photometricStereo(imageList, intList, lightMat, mask, pathToAmbiant, PSParameters, normals, albedo);

    writePSResults(outputPath, normals, albedo);
    image::writeImage(outputPath + "/mask.png", mask, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));
}

void photometricStereo(const sfmData::SfMData& sfmData, const std::string& lightData, const std::string& maskPath, const std::string& outputPath, const PhotometricSteroParameters& PSParameters, image::Image<image::RGBfColor>& normals, image::Image<image::RGBfColor>& albedo)
{
    size_t dim = 3;
    if(PSParameters.SHOrder == 2)
    {
        dim = 9;
    }

    std::string pathToAmbiant = "";
    std::map<IndexT, std::vector<IndexT>> viewsPerPoseId;

    for(auto& viewIt: sfmData.getViews())
    {
        viewsPerPoseId[viewIt.second->getPoseId()].push_back(viewIt.second->getViewId());
    }

    for(auto& posesIt: viewsPerPoseId)
    {
        std::vector<std::string> imageList;

        ALICEVISION_LOG_INFO("Pose Id: " << posesIt.first);
        std::vector<IndexT>& initViewIds = posesIt.second;

        std::map<std::string, IndexT> idMap;
        for(auto& viewId: initViewIds)
        {
            std::map<std::string, std::string> currentMetadata = sfmData.getView(viewId).getMetadata();
            idMap[currentMetadata.at("Exif:DateTimeDigitized")] = viewId;
        }

        std::vector<IndexT> viewIds;
        for(const auto& [currentTime, viewId] : idMap)
        {
            viewIds.push_back(viewId);
        }


        for(auto& viewId: viewIds)
        {
            const fs::path imagePath = fs::path(sfmData.getView(viewId).getImagePath());
            if(!boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
            {
                ALICEVISION_LOG_INFO("  - " << imagePath.string());
                imageList.push_back(imagePath.string());
            }
            else if(PSParameters.removeAmbiant)
            {
                pathToAmbiant = imagePath.string();
            }
        }

        std::vector<std::array<float, 3>> intList; // Light intensities
        Eigen::MatrixXf lightMat(imageList.size(), dim); //Light directions

        if(fs::is_directory(lightData))    // #pragma omp parallel for
        {
            loadPSData(lightData, PSParameters.SHOrder, intList, lightMat);
        }
        else
        {
            buildLigtMatFromJSON(lightData, viewIds, lightMat, intList);
        }

        image::Image<float> mask;
        std::string pictureFolderName = fs::path(sfmData.getView(viewIds[0]).getImagePath()).parent_path().filename().string();
        std::string currentMaskPath = maskPath + "/" + pictureFolderName.erase(0,3) + ".png";
        loadMask(currentMaskPath, mask);

        photometricStereo(imageList, intList, lightMat, mask, pathToAmbiant, PSParameters, normals, albedo);

        writePSResults(outputPath, normals, albedo, posesIt.first);
        image::writeImage(outputPath + "/" + std::to_string(posesIt.first) + "_mask.png", mask, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));

        if(sfmData.getPoses().size() > 0)
        {
            const Mat3 rotation = sfmData.getPose(sfmData.getView(viewIds[0])).getTransform().rotation().transpose();
            applyRotation(rotation, normals);
        }

        image::Image<image::RGBColor> normalsImPNG(normals.cols(),normals.rows());
        convertNormalMap2png(normals, normalsImPNG);

        image::writeImage(outputPath + "/" + std::to_string(posesIt.first) + "_normals_w.png", normalsImPNG, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
        image::writeImage(outputPath + "/" + std::to_string(posesIt.first) + "_normals_w.exr", normals, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
    }

    sfmData::SfMData AlbedoSfmData = sfmData;

    std::set<IndexT> viewIdsToRemove;
    // Create Albedo SfmData :
    for(auto& viewIt: AlbedoSfmData.getViews())
    {
        const IndexT viewId = viewIt.first;
        IndexT poseId = viewIt.second->getPoseId();

        if(viewId == poseId)
        {
            sfmData::View * view = AlbedoSfmData.getViews().at(viewId).get();
            std::string imagePath = outputPath + "/" + std::to_string(poseId) + "_albedo.exr";
            view->setImagePath(imagePath);
        }
        else
        {
            viewIdsToRemove.insert(viewId);
        }
    }
    for (auto r : viewIdsToRemove)
        AlbedoSfmData.getViews().erase(r);

    sfmDataIO::Save(AlbedoSfmData, outputPath + "/albedoMaps.sfm", sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS));

    // Create Normal SfmData
    sfmData::SfMData NormalSfmData = AlbedoSfmData;
    for(auto& viewIt: NormalSfmData.getViews())
    {
        const IndexT viewId = viewIt.first;
        IndexT poseId = viewIt.second->getPoseId();

        sfmData::View * view = NormalSfmData.getViews().at(viewId).get();
        std::string imagePath = outputPath + "/" + std::to_string(poseId) + "_normals_w.exr";
        view->setImagePath(imagePath);
    }

    sfmDataIO::Save(NormalSfmData, outputPath + "/normalMaps.sfm", sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS));
}

void photometricStereo(const std::vector<std::string>& imageList, const std::vector<std::array<float, 3>>& intList, const Eigen::MatrixXf& lightMat, image::Image<float>& mask, const std::string& pathToAmbiant, const PhotometricSteroParameters& PSParameters, image::Image<image::RGBfColor>& normals, image::Image<image::RGBfColor>& albedo)
{
    size_t maskSize;
    int pictRows;
    int pictCols;

    bool hasMask = !((mask.rows() == 1) && (mask.cols() == 1));

    std::string picturePath;
    std::string pictureName;

    std::vector<int> indexes;

    if(hasMask)
    {
        if(PSParameters.downscale > 1)
        {
            downscaleImageInplace(mask,PSParameters.downscale);
        }

        getIndMask(mask, indexes);
        maskSize = indexes.size();
        pictRows = mask.rows();
        pictCols = mask.cols();
    }
    else
    {
        picturePath = imageList.at(0);

        image::Image<image::RGBfColor> imageFloat;
        image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

        if(PSParameters.downscale > 1)
        {
            downscaleImageInplace(imageFloat,PSParameters.downscale);
        }

        pictRows = imageFloat.rows();
        pictCols = imageFloat.cols();

        maskSize = pictRows*pictCols;
    }

    Eigen::MatrixXf imMat(3*imageList.size(), maskSize);
    Eigen::MatrixXf imMat_gray(imageList.size(), maskSize);

    // Read pictures :
    image::Image<image::RGBfColor> imageAmbiant;

    if(boost::algorithm::icontains(fs::path(pathToAmbiant).stem().string(), "ambiant"))
    {
        std::cout << "Removing ambiant light" << std::endl;
        std::cout << pathToAmbiant << std::endl;

        image::readImage(pathToAmbiant, imageAmbiant, image::EImageColorSpace::NO_CONVERSION);

        if(PSParameters.downscale > 1)
        {
            downscaleImageInplace(imageAmbiant,PSParameters.downscale);
        }
    }

    for (size_t i = 0; i < imageList.size(); ++i)
    {
        picturePath = imageList.at(i);

        image::Image<image::RGBfColor> imageFloat;
        image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

        if(PSParameters.downscale > 1)
        {
            downscaleImageInplace(imageFloat,PSParameters.downscale);
        }

        if(boost::algorithm::icontains(fs::path(pathToAmbiant).stem().string(), "ambiant"))
        {
            imageFloat = imageFloat - imageAmbiant;
        }

        intensityScaling(intList.at(i), imageFloat);

        Eigen::MatrixXf currentPicture(3,maskSize);
        image2PsMatrix(imageFloat, mask, currentPicture);


        imMat.block(3*i,0,3,maskSize) = currentPicture;
        imMat_gray.block(i,0,1,maskSize) = currentPicture.block(0,0,1,maskSize) * 0.2126 + currentPicture.block(1,0,1,maskSize) * 0.7152 + currentPicture.block(2,0,1,maskSize) * 0.0722;
    }

    imMat = imMat/imMat.maxCoeff();
    imMat_gray = imMat_gray/imMat_gray.maxCoeff();

    Eigen::MatrixXf normalsVect = Eigen::MatrixXf::Zero(lightMat.cols(),pictRows*pictCols);
    Eigen::MatrixXf albedoVect = Eigen::MatrixXf::Zero(3,pictRows*pictCols);
    Eigen::MatrixXf M_channel(3, maskSize);

    // Normal estimation :
    M_channel = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(imMat_gray);
    int currentIdx;

    for (size_t i = 0; i < maskSize; ++i)
    {
        if(hasMask)
        {
            currentIdx = indexes.at(i); // index in picture
        }
        else
        {
            currentIdx = i;
        }
        normalsVect.col(currentIdx) = M_channel.col(i)/M_channel.col(i).norm();
    }
    if(PSParameters.isRobust)
    {
        float mu = 0.1;
        int max_iterations = 1000;
        float epsilon = 0.001;

        // Errors (E) and Lagrange multiplicators (W) initialisation
        Eigen::MatrixXf E = lightMat*M_channel - imMat_gray;
        Eigen::MatrixXf W = Eigen::MatrixXf::Zero(E.rows(), E.cols());

        Eigen::MatrixXf M_kminus1;
        Eigen::MatrixXf newImMat;

        for (size_t k = 0; k < max_iterations; ++k)
        {
            // Copy for convergence test :
            M_kminus1 = M_channel;

            // M update :
            newImMat = imMat_gray + E - W/mu;
            M_channel = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(newImMat);

            // E update :
            Eigen::MatrixXf E_before = E;
            shrink(lightMat*M_channel-imMat_gray + W/mu, 1.0/mu, E);

            // W update :
            W = W + mu*(lightMat*M_channel-imMat_gray-E);

            // Convergence test :
            Eigen::MatrixXf ecart = M_kminus1 - M_channel;
            float ecart_relatif = ecart.norm()/M_channel.norm();

            if (k > 10 && ecart_relatif < epsilon)
            {
                std::cout << k << std::endl;
                std::cout << "Convergence" << std::endl;
                break;
            }
        }

        for (size_t i = 0; i < maskSize; ++i)
        {
            if(hasMask)
            {
                currentIdx = indexes.at(i); // index in picture
            }
            else
            {
                currentIdx = i;
            }
            normalsVect.col(currentIdx) = M_channel.col(i)/M_channel.col(i).norm();
        }


        int currentIdx;
        for (size_t ch = 0; ch < 3; ++ch)
        {
            // Create I matrix for current pixel :
            Eigen::MatrixXf pixelValues_channel(imageList.size(), maskSize);
            for (size_t i = 0; i < imageList.size(); ++i)
            {
                pixelValues_channel.block(i, 0, 1, maskSize) = imMat.block(ch + 3*i, 0, 1, maskSize);
            }

            for (size_t i = 0; i < maskSize; ++i)
            {
                if(hasMask)
                {
                    currentIdx = indexes.at(i); // index in picture
                }
                else
                {
                    currentIdx = i;
                }
                Eigen::VectorXf currentI = pixelValues_channel.col(i);
                Eigen::VectorXf currentShading = lightMat*normalsVect.col(currentIdx);
                Eigen::VectorXf result = currentI.cwiseProduct(currentShading.cwiseInverse());
                median(result, albedoVect(ch, currentIdx));
            }
        }
    }
    else
    {
        // Channelwise albedo estimation :
        for (size_t ch = 0; ch < 3; ++ch)
        {
            // Create I matrix for current pixel :
            Eigen::MatrixXf pixelValues_channel(imageList.size(), maskSize);
            for (size_t i = 0; i < imageList.size(); ++i)
            {
                pixelValues_channel.block(i, 0, 1, maskSize) = imMat.block(ch + 3*i, 0, 1, maskSize);
            }

            M_channel = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(pixelValues_channel);

            for (size_t i = 0; i < maskSize; ++i)
            {
                if(hasMask)
                {
                    currentIdx = indexes.at(i); // index in picture
                }
                else
                {
                    currentIdx = i;
                }
                albedoVect(ch, currentIdx) = M_channel.col(i).norm();
            }
        }
    }

    albedoVect = albedoVect/albedoVect.maxCoeff();
    image::Image<image::RGBfColor> normalsIm(pictCols,pictRows);
    reshapeInImage(normalsVect, normalsIm);
    normals = normalsIm;

    image::Image<image::RGBfColor> albedoIm(pictCols,pictRows);
    reshapeInImage(albedoVect, albedoIm);
    albedo = albedoIm;

}

void loadPSData(const std::string& folderPath, const size_t HS_order, std::vector<std::array<float, 3>>& intList, Eigen::MatrixXf& lightMat)
{
    std::string intFileName;
    std::string pathToCM;
    std::string dirFileName;

    // Light instensities :
    intFileName = folderPath + "/light_intensities.txt";
    loadLightIntensities(intFileName, intList);

    // Convertion matrix :
    Eigen::MatrixXf convertionMatrix = Eigen::Matrix<float, 3, 3>::Identity();
    pathToCM = folderPath + "/convertionMatrix.txt";
    if(fs::exists(pathToCM))
    {
        readMatrix(pathToCM, convertionMatrix);
    }

    // Light directions :
    if(HS_order == 0)
    {
        dirFileName = folderPath + "/light_directions.txt";
        loadLightDirections(dirFileName, convertionMatrix, lightMat);
    } else if (HS_order == 2) {
        dirFileName = folderPath + "/light_directions_HS.txt";
        loadLightHS(dirFileName, lightMat);
    }
}

void getPicturesNames(const std::string& folderPath, std::vector<std::string>& imageList)
{
    const std::vector<std::string>& extensions = image::getSupportedExtensions();

    fs::directory_iterator endItr;
    for(fs::directory_iterator itr(folderPath); itr != endItr; ++itr)
    {
        fs::path currentFilePath = itr->path();

        std::string fileExtension = fs::extension(currentFilePath.string());
        std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(), ::tolower);

        if(!boost::algorithm::icontains(currentFilePath.stem().string(), "mask") && !boost::algorithm::icontains(currentFilePath.stem().string(), "ambiant"))
        {
            for(const std::string& extension: extensions)
            {
                if(fileExtension == extension)
                {
                    imageList.push_back(currentFilePath.string());
                }
            }
        }
    }

    std::sort(imageList.begin(),imageList.end(),compareFunction); //sort the vector
}

bool compareFunction(std::string a, std::string b) {return a<b;}

void shrink(const Eigen::MatrixXf& mat, const float rho, Eigen::MatrixXf& E)
{
    for (size_t i = 0; i < E.rows(); ++i)
    {
        for (size_t j = 0; j < E.cols(); ++j)
        {
            if(mat(i,j) > 0)
            {
                E(i,j) = std::max(std::abs(mat(i,j))-rho, float(0.0));
            }
            else
            {
                E(i,j) = -std::max(std::abs(mat(i,j))-rho, float(0.0));
            }
        }
    }
}

void median(const Eigen::MatrixXf& d, float& median){
    Eigen::MatrixXf aux = d;
    std::sort(aux.data(),aux.data()+aux.size());
    size_t middle = aux.size()/2;
    aux.size() % 2 == 0 ?
        median = aux((aux.size()-1)/2) + aux((aux.size()+1)/2) :
        median = aux(middle);
}



}

void applyRotation(const Eigen::MatrixXd& rotation, image::Image<image::RGBfColor>& normals)
{
    for (int i = 0; i < normals.rows(); ++i)
    {
        for (int j = 0; j < normals.cols(); ++j)
        {
            normals(i,j)(0) = rotation(0,0)*normals(i,j)(0) + rotation(0,1)*normals(i,j)(1) + rotation(0,2)*normals(i,j)(2);
            normals(i,j)(1) = rotation(1,0)*normals(i,j)(0) + rotation(1,1)*normals(i,j)(1) + rotation(1,2)*normals(i,j)(2);
            normals(i,j)(2) = rotation(2,0)*normals(i,j)(0) + rotation(2,1)*normals(i,j)(1) + rotation(2,2)*normals(i,j)(2);
        }
    }

}

}
