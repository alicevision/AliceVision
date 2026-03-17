#include "lightingData.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <filesystem>

#include <aliceVision/system/Logger.hpp>

namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

namespace aliceVision {
namespace lightingEstimation {

Lighting::Lighting(LightType lightType_)
  : lightType(lightType_)
{}

LightType Lighting::getLightType() const
{ return lightType; }

/*
 * Directionnal lighting
 */
DirectionnalLighting::DirectionnalLighting(const Eigen::Vector3f& lightDirection_, const std::vector<float>& intensity_)
  : Lighting(LightType::Directionnal),
    lightDirection(lightDirection_),
    intensity(intensity_)
{}

DirectionnalLighting::DirectionnalLighting(const Eigen::Vector3f& lightDirection_, const Eigen::Vector3f& intensity_)
  : Lighting(LightType::Directionnal),
    lightDirection(lightDirection_),
    intensity(3, 0)
{
    intensity[0] = intensity_(0);
    intensity[1] = intensity_(1);
    intensity[2] = intensity_(2);
}

DirectionnalLighting::DirectionnalLighting(const Eigen::Vector3f& lightDirection_, float intensity_)
  : Lighting(LightType::Directionnal),
    lightDirection(lightDirection_),
    intensity(3, intensity_)
{}

DirectionnalLighting::~DirectionnalLighting()
{}

DirectionnalLighting* DirectionnalLighting::clone() const { return new DirectionnalLighting(*this); }

DirectionnalLighting* DirectionnalLighting::convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const
{
	Eigen::Matrix4f av_to_vis = Eigen::Matrix4f::Identity();
	av_to_vis(1, 1) = -1.;
	av_to_vis(2, 2) = -1.;
	Eigen::Matrix4f RT = pose->getTransform().getHomogeneous().cast<float>() * av_to_vis;

    Eigen::Vector3f newLightDirection = RT.block<3, 3>(0, 0) * lightDirection;

    return new DirectionnalLighting(newLightDirection, intensity);
}

const Eigen::Vector3f& DirectionnalLighting::getLightDirection() const { return lightDirection; }

const std::vector<float>& DirectionnalLighting::getLightIntensity() const { return intensity; }


/*
 * Punctual lighting
 */
PunctualLighting::PunctualLighting(const Eigen::Vector3f& lightPosition_, const std::vector<float>& intensity_)
  : Lighting(LightType::Punctual),
    lightPosition(lightPosition_),
    intensity(intensity_)
{}

PunctualLighting::PunctualLighting(const Eigen::Vector3f& lightPosition_, const Eigen::Vector3f& intensity_)
  : Lighting(LightType::Punctual),
    lightPosition(lightPosition_),
    intensity(3, 0)
{
    intensity[0] = intensity_(0);
    intensity[1] = intensity_(1);
    intensity[2] = intensity_(2);
}

PunctualLighting::PunctualLighting(const Eigen::Vector3f& lightPosition_, float intensity_)
  : Lighting(LightType::Punctual),
    lightPosition(lightPosition_),
    intensity(3, intensity_)
{}

PunctualLighting::~PunctualLighting()
{}

PunctualLighting* PunctualLighting::clone() const { return new PunctualLighting(*this); }

PunctualLighting* PunctualLighting::convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const
{
	Eigen::Matrix4f av_to_vis = Eigen::Matrix4f::Identity();
	av_to_vis(1, 1) = -1.;
	av_to_vis(2, 2) = -1.;
	Eigen::Matrix4f RT = pose->getTransform().getHomogeneous().cast<float>() * av_to_vis;

    Eigen::Vector3f newLightPosition = RT.block<3, 3>(0, 0) * lightPosition + RT.block<3,1>(3,0);

    return new PunctualLighting(newLightPosition, intensity);
}

const Eigen::Vector3f& PunctualLighting::getLightPosition() const { return lightPosition; }

const std::vector<float>& PunctualLighting::getLightIntensity() const { return intensity; }


/*
 * LED lighting
 */
LEDLighting::LEDLighting(
      const Eigen::Vector3f& lightPosition_, 
      const Eigen::Vector3f& lightDirection_, 
      const Eigen::Vector3f& lightRGBIntensity_, 
      const Eigen::Vector3f& lightRGBAnisotropy_): 
    Lighting(LightType::LED),
    lightPosition(lightPosition_),
    lightDirection(lightDirection_),
    lightRGBIntensity(lightRGBIntensity_),
    lightRGBAnisotropy(lightRGBAnisotropy_)
{}

LEDLighting::~LEDLighting()
{}

LEDLighting* LEDLighting::clone() const { return new LEDLighting(*this); }

LEDLighting* LEDLighting::convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const
{
	Eigen::Matrix4f av_to_vis = Eigen::Matrix4f::Identity();
	av_to_vis(1, 1) = -1.;
	av_to_vis(2, 2) = -1.;
	Eigen::Matrix4f RT = pose->getTransform().getHomogeneous().cast<float>() * av_to_vis;

    Eigen::Vector3f newLightPosition = RT.block<3, 3>(0, 0) * lightPosition + RT.block<3,1>(3,0);
    Eigen::Vector3f newLightDirection = RT.block<3, 3>(0, 0) * lightDirection;

    return new LEDLighting(newLightPosition, newLightDirection, lightRGBIntensity, lightRGBAnisotropy);
}

const Eigen::Vector3f& LEDLighting::getLightPosition() const { return lightPosition; }

const Eigen::Vector3f& LEDLighting::getLightDirection() const { return lightDirection; }

const Eigen::Vector3f& LEDLighting::getLightRGBIntensity() const { return lightRGBIntensity; }

const Eigen::Vector3f& LEDLighting::getLightRGBAnisotropy() const { return lightRGBAnisotropy; }



/*
 * IO functions
 */
namespace LightingDataIO {
bool saveJSON(const Lightings& lightings, const std::string& filename)
{
    bpt::ptree lightsTree;
    bpt::ptree fileTree;

    ALICEVISION_LOG_INFO("Building tree from lightings");
    for (auto lightIt = lightings.begin(); lightIt != lightings.end(); lightIt++)
    {
        auto lightId = lightIt->first;

        bpt::ptree lightTree;

        lightTree.put("lightId", lightId);
        if (lightIt->second->getLightType() == LightType::Directionnal)
        {
            std::shared_ptr<DirectionnalLighting> dirLighting = std::static_pointer_cast<DirectionnalLighting>(lightIt->second);
            if (!dirLighting)
            {
                ALICEVISION_LOG_ERROR("Wrong lighting type");
                return false;
            }

            // Light type
            lightTree.put("type", "directionnal");

            // Light direction
            bpt::ptree directionNode;
            int dirLightSize = dirLighting->getLightDirection().size();
            for (int i = 0; i < dirLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(dirLighting->getLightDirection()(i));
                directionNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("direction", directionNode);

            // Light intensity
            bpt::ptree intensityNode;
            int intLightSize = dirLighting->getLightIntensity().size();
            for (unsigned int i = 0; i < intLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(dirLighting->getLightIntensity()[i]);
                intensityNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("intensity", intensityNode);
        }
        else if (lightIt->second->getLightType() == LightType::Punctual)
        {
            std::shared_ptr<PunctualLighting> pointLighting = std::static_pointer_cast<PunctualLighting>(lightIt->second);
            if (!pointLighting)
            {
                ALICEVISION_LOG_ERROR("Wrong lighting type");
                return false;
            }

            // Light type
            lightTree.put("type", "punctual");

            // Light position 
            bpt::ptree positionNode;
            int posLightSize = pointLighting->getLightPosition().size();
            for (int i = 0; i < posLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(pointLighting->getLightPosition()(i));
                positionNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("position", positionNode);

            // Light intensity
            bpt::ptree intensityNode;
            int intLightSize = pointLighting->getLightIntensity().size();
            for (unsigned int i = 0; i < intLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(pointLighting->getLightIntensity()[i]);
                intensityNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("intensity", intensityNode);
        }
        else if (lightIt->second->getLightType() == LightType::LED)
        {
            std::shared_ptr<LEDLighting> pointLighting = std::static_pointer_cast<LEDLighting>(lightIt->second);
            if (!pointLighting)
            {
                ALICEVISION_LOG_ERROR("Wrong lighting type");
                return false;
            }

            // Light type
            lightTree.put("type", "led");

            // Light position 
            bpt::ptree positionNode;
            int posLightSize = pointLighting->getLightPosition().size();
            for (int i = 0; i < posLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(pointLighting->getLightPosition()(i));
                positionNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("position", positionNode);

            // Light direction 
            bpt::ptree directionNode;
            int dirLightSize = pointLighting->getLightDirection().size();
            for (int i = 0; i < dirLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(pointLighting->getLightDirection()(i));
                directionNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("direction", directionNode);

            // Light rgb intensity
            bpt::ptree rgbIntensityNode;
            int rgbIntensityLightSize = pointLighting->getLightRGBIntensity().size();
            for (int i = 0; i < rgbIntensityLightSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(pointLighting->getLightRGBIntensity()(i));
                rgbIntensityNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("intensity", rgbIntensityNode);

            // Light rgb anisotropy
            bpt::ptree rgbAnisotropyNode;
            int rgbAnisotropySize = pointLighting->getLightRGBAnisotropy().size();
            for (int i = 0; i < rgbAnisotropySize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(pointLighting->getLightRGBAnisotropy()(i));
                rgbAnisotropyNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("anisotropy", rgbAnisotropyNode);
        }
		else
		{
			ALICEVISION_LOG_ERROR("Light type not handled");
			return false;
		}

        lightsTree.push_back(std::make_pair("", lightTree));
    }

    fileTree.add_child("lights", lightsTree);
    bpt::write_json(filename, fileTree);
    return true;
}

bool loadJSON(const std::string& filename, Lightings& lightings)
{
    // Main tree
    bpt::ptree fileTree;
    // Read the json file and initialize the tree
    bpt::read_json(filename, fileTree);
    
    // iterator on spheres
    auto& lights = fileTree.get_child("lights");
	for (auto itLight = lights.get_child("").begin(); itLight != lights.get_child("").end(); itLight++)
	{
		IndexT lightId = stoi(itLight->second.get("lightId", "0"));

		std::string lightType = itLight->second.get("type", "None");

		if (lightType == "directionnal")
		{
			std::vector<float> currentIntensities;
			std::vector<float> currentDirection;
			for (auto& intensities : itLight->second.get_child("intensity"))
			{
				currentIntensities.push_back(intensities.second.get_value<float>());
			}

			for (auto& direction : itLight->second.get_child("direction"))
			{
				currentDirection.push_back(direction.second.get_value<float>());
			}

			if (currentDirection.size() != 3)
			{
				ALICEVISION_LOG_ERROR("Directionnal lighting should have 3 components (Spherical Harmonics not handled)");
				return false;
			}
            auto lighting = std::make_shared<DirectionnalLighting>(
                Eigen::Vector3f(currentDirection[0], currentDirection[1], currentDirection[2]), currentIntensities);
            lightings.emplace(lightId, lighting);
        }
        else if (lightType == "punctual")
		{
			std::vector<float> currentIntensities;
			std::vector<float> currentPosition;
			for (auto& intensities : itLight->second.get_child("intensity"))
			{
				currentIntensities.push_back(intensities.second.get_value<float>());
			}

			for (auto& position: itLight->second.get_child("position"))
			{
				currentPosition.push_back(position.second.get_value<float>());
			}

			if (currentPosition.size() != 3)
			{
				ALICEVISION_LOG_ERROR("Lighting position should have 3 components");
				return false;
			}
            auto lighting = std::make_shared<PunctualLighting>(
                Eigen::Vector3f(currentPosition[0], currentPosition[1], currentPosition[2]), currentIntensities);
            lightings.emplace(lightId, lighting);
		}
        else if (lightType == "led")
		{
			std::vector<float> currentPosition;
			std::vector<float> currentDirection;
			std::vector<float> currentRGBIntensity;
			std::vector<float> currentRGBAnisotropy;

			for (auto& position: itLight->second.get_child("position"))
			{
				currentPosition.push_back(position.second.get_value<float>());
			}

			for (auto& direction: itLight->second.get_child("direction"))
			{
				currentDirection.push_back(direction.second.get_value<float>());
			}

			for (auto& rgbintensity : itLight->second.get_child("intensity"))
			{
				currentRGBIntensity.push_back(rgbintensity.second.get_value<float>());
			}

			for (auto& rgbanisotropy : itLight->second.get_child("anisotropy"))
			{
				currentRGBAnisotropy.push_back(rgbanisotropy.second.get_value<float>());
			}

			if (currentPosition.size() != 3)
			{
				ALICEVISION_LOG_ERROR("Lighting position should have 3 components");
				return false;
			}
            auto lighting = std::make_shared<LEDLighting>(
                Eigen::Vector3f(currentPosition[0], currentPosition[1], currentPosition[2]), 
                Eigen::Vector3f(currentDirection[0], currentDirection[1], currentDirection[2]), 
                Eigen::Vector3f(currentRGBIntensity[0], currentRGBIntensity[1], currentRGBIntensity[2]), 
                Eigen::Vector3f(currentRGBAnisotropy[0], currentRGBAnisotropy[1], currentRGBAnisotropy[2])
            );
            lightings.emplace(lightId, lighting);
		}
		else
		{
			ALICEVISION_LOG_ERROR("Light type " << lightType << " not handled");
			return false;
		}
	}
	return true;
}

}  // namespace LightingDataIO
}  // namespace lightingEstimation
}  // namespace aliceVision
