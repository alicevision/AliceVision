#pragma once

#include <Eigen/Dense>

#include <aliceVision/sfmData/SharedPtrMap.hpp>
#include <aliceVision/sfmData/CameraPose.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision {
namespace lightingEstimation {

enum class LightType
{
    Directionnal,
    Punctual
};

class Lighting
{
  public:
    Lighting(LightType lightType_);

    virtual ~Lighting() = default;

    virtual Lighting* clone() const = 0;
    
    virtual Lighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const = 0;

    LightType getLightType() const;

  private:
    LightType lightType;
};

class DirectionnalLighting : public Lighting
{
  public:
    DirectionnalLighting(const Eigen::Vector3f& lightDirection_, const std::vector<float>& intensity_);

    DirectionnalLighting(const Eigen::Vector3f& lightDirection_, float intensity_);

    ~DirectionnalLighting();

    DirectionnalLighting* clone() const;

    DirectionnalLighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const;

    const Eigen::Vector3f& getLightDirection() const;

    const std::vector<float>& getLightIntensity() const;

  private:
    Eigen::Vector3f lightDirection;
    std::vector<float> intensity;
};

class PunctualLighting : public Lighting
{
  public:
    PunctualLighting(const Eigen::Vector3f& lightPosition_, const std::vector<float>& intensity_);

    PunctualLighting(const Eigen::Vector3f& lightPosition_, float intensity_);

    ~PunctualLighting();

    PunctualLighting* clone() const;

    PunctualLighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const;

    const Eigen::Vector3f& getLightPosition() const;

    const std::vector<float>& getLightIntensity() const;

  private:
    Eigen::Vector3f lightPosition;
    std::vector<float> intensity;
};


using Lightings = sfmData::SharedPtrMap<Lighting>;


namespace LightingDataIO {
bool saveJSON(const Lightings& lightings, const std::string& filename);

bool loadJSON(const std::string& filename, Lightings& lightings);
}

}  // namespace lightingEstimation
}  // namespace aliceVision
