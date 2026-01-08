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


DirectionnalLighting::DirectionnalLighting(const Eigen::Vector3f& lightDirection_, const std::vector<float>& intensity_)
  : Lighting(LightType::Directionnal),
    lightDirection(lightDirection_),
    intensity(intensity_)
{}

DirectionnalLighting::DirectionnalLighting(const Eigen::Vector3f& lightDirection_, float intensity_)
  : Lighting(LightType::Directionnal),
    lightDirection(lightDirection_),
    intensity(3, intensity_)
{}

DirectionnalLighting::~DirectionnalLighting()
{}

const Eigen::Vector3f& DirectionnalLighting::getLightDirection() const { return lightDirection; }

const std::vector<float>& DirectionnalLighting::getLightIntensity() const { return intensity; }

namespace LightingDataIO {
bool saveJSON(const Lightings& lightings, const std::string& filename)
{
    bpt::ptree lightsTree;
    bpt::ptree fileTree;

    int imgCpt = 0;

    ALICEVISION_LOG_INFO("Building tree from lightings");
    for (auto lightIt = lightings.begin(); lightIt != lightings.end(); lightIt++)
    {
        auto lightId = lightIt->first;

        bpt::ptree lightTree;

        lightTree.put("lightId", imgCpt);
        if (lightIt->second->getLightType() == LightType::Directionnal)
        {
            std::shared_ptr<DirectionnalLighting> dirLighting = std::static_pointer_cast<DirectionnalLighting>(lightIt->second);
            if (!dirLighting)
            {
                ALICEVISION_LOG_ERROR("Wrong lightings type");
                return false;
            }

            // Light type
            lightTree.put("lightType", "Directionnal");

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

		std::string lightType = itLight->second.get("lightType", "None");

		if (lightType == "Directionnal")
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
			auto lighting = std::make_shared<DirectionnalLighting>(Eigen::Vector3f(currentDirection[0], currentDirection[1], currentDirection[2]),
																   currentIntensities);
			lightings.emplace(lightId, lighting);
		}
		else
		{
			ALICEVISION_LOG_ERROR("LightType " << lightType << " not handled");
			return false;
		}
	}
	return true;
}

}  // namespace LightingDataIO
}  // namespace lightingEstimation
}  // namespace aliceVision
