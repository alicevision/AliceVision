#pragma once

#include <Eigen/Dense>

#include <aliceVision/sfmData/SharedPtrMap.hpp>
#include <aliceVision/sfmData/CameraPose.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision {
namespace lightingEstimation {

/**
 * Supported lighting models handled by the lighting estimation module.
 */
enum class LightType
{
    Directionnal,
    Punctual,
    LED
};

/**
 * Base interface for a light source description.
 */
class Lighting
{
  public:
    /**
     * Construct a light with the given runtime type.
     * @param[in] lightType_ Concrete light model identifier stored by the instance.
     */
    Lighting(LightType lightType_);

    /**
     * Virtual destructor for polymorphic ownership.
     */
    virtual ~Lighting() = default;

    /**
     * Create an owning copy of the current light instance.
     */
    virtual Lighting* clone() const = 0;
    
    /**
     * Express the light parameters in the coordinate frame defined by a camera pose.
     * @param[in] pose Camera pose defining the destination coordinate frame.
     */
    virtual Lighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const = 0;

    /**
     * Return the concrete light model type.
     */
    LightType getLightType() const;

  private:
    LightType lightType;
};

/**
 * Directional light represented by a direction and RGB intensity.
 */
class DirectionnalLighting : public Lighting
{
  public:
    /**
     * Construct a directional light from a direction and per-channel intensity values.
     * @param[in] lightDirection_ Light direction vector.
     * @param[in] intensity_ Per-channel light intensity values.
     */
    DirectionnalLighting(const Eigen::Vector3f& lightDirection_, const std::vector<float>& intensity_);

    /**
     * Construct a directional light with a shared scalar intensity for all channels.
     * @param[in] lightDirection_ Light direction vector.
     * @param[in] intensity_ Scalar intensity copied to all channels.
     */
    DirectionnalLighting(const Eigen::Vector3f& lightDirection_, float intensity_);

    /**
     * Construct a directional light from a direction and RGB intensity vector.
     * @param[in] lightDirection_ Light direction vector.
     * @param[in] intensity_ RGB light intensity vector.
     */
    DirectionnalLighting(const Eigen::Vector3f& lightDirection_, const Eigen::Vector3f& intensity_);

    /**
     * Destroy the directional light instance.
     */
    ~DirectionnalLighting();

    /**
     * Duplicate this directional light.
     */
    DirectionnalLighting* clone() const;

    /**
     * Convert the light direction into the frame defined by a camera pose.
     * @param[in] pose Camera pose defining the destination coordinate frame.
     */
    DirectionnalLighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const;

    /**
     * Return the light direction.
     */
    const Eigen::Vector3f& getLightDirection() const;

    /**
     * Return the per-channel light intensity.
     */
    const std::vector<float>& getLightIntensity() const;

  private:
    Eigen::Vector3f lightDirection;
    std::vector<float> intensity;
};

/**
 * Point light represented by a 3D position and RGB intensity.
 */
class PunctualLighting : public Lighting
{
  public:
    /**
     * Construct a point light from a position and per-channel intensity values.
     * @param[in] lightPosition_ Light position in 3D space.
     * @param[in] intensity_ Per-channel light intensity values.
     */
    PunctualLighting(const Eigen::Vector3f& lightPosition_, const std::vector<float>& intensity_);

    /**
     * Construct a point light from a position and RGB intensity vector.
     * @param[in] lightPosition_ Light position in 3D space.
     * @param[in] intensity_ RGB light intensity vector.
     */
    PunctualLighting(const Eigen::Vector3f& lightPosition_, const Eigen::Vector3f& intensity_);

    /**
     * Construct a point light with a shared scalar intensity for all channels.
     * @param[in] lightPosition_ Light position in 3D space.
     * @param[in] intensity_ Scalar intensity copied to all channels.
     */
    PunctualLighting(const Eigen::Vector3f& lightPosition_, float intensity_);

    /**
     * Destroy the point light instance.
     */
    ~PunctualLighting();

    /**
     * Duplicate this point light.
     */
    PunctualLighting* clone() const;

    /**
     * Convert the light position into the frame defined by a camera pose.
     * @param[in] pose Camera pose defining the destination coordinate frame.
     */
    PunctualLighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const;

    /**
     * Return the light position.
     */
    const Eigen::Vector3f& getLightPosition() const;

    /**
     * Return the per-channel light intensity.
     */
    const std::vector<float>& getLightIntensity() const;

  private:
    Eigen::Vector3f lightPosition;
    std::vector<float> intensity;
};

/**
 * LED light with position, direction, RGB intensity, and RGB anisotropy.
 */
class LEDLighting : public Lighting
{
  public:
    /**
     * Construct an LED light from its spatial and radiometric parameters.
     * @param[in] lightPosition_ Light position in 3D space.
     * @param[in] lightDirection_ Light emission direction vector.
     * @param[in] lightRGBIntensity_ RGB light intensity vector.
     * @param[in] lightRGBAnisotropy_ RGB anisotropy coefficients.
     */
    LEDLighting(
      const Eigen::Vector3f& lightPosition_, 
      const Eigen::Vector3f& lightDirection_, 
      const Eigen::Vector3f& lightRGBIntensity_, 
      const Eigen::Vector3f& lightRGBAnisotropy_);

    /**
     * Destroy the LED light instance.
     */
    ~LEDLighting();

    /**
     * Duplicate this LED light.
     */
    LEDLighting* clone() const;

    /**
     * Convert the LED position and direction into the frame defined by a camera pose.
     * @param[in] pose Camera pose defining the destination coordinate frame.
     */
    LEDLighting* convertToFrame(const std::shared_ptr<sfmData::CameraPose>& pose) const;

    /**
     * Return the light position.
     */
    const Eigen::Vector3f& getLightPosition() const;

    /**
     * Return the light direction.
     */
    const Eigen::Vector3f& getLightDirection() const;

    /**
     * Return the RGB intensity.
     */
    const Eigen::Vector3f& getLightRGBIntensity() const;

    /**
     * Return the RGB anisotropy coefficients.
     */
    const Eigen::Vector3f& getLightRGBAnisotropy() const;

  private:
    Eigen::Vector3f lightPosition;
    Eigen::Vector3f lightDirection;
    Eigen::Vector3f lightRGBIntensity;
    Eigen::Vector3f lightRGBAnisotropy;
};



using Lightings = sfmData::SharedPtrMap<Lighting>;


namespace LightingDataIO {
/**
 * Save a lighting collection to a JSON file.
 * @param[in] lightings Lighting collection to serialize.
 * @param[in] filename Destination JSON file path.
 */
bool saveJSON(const Lightings& lightings, const std::string& filename);

/**
 * Load a lighting collection from a JSON file.
 * @param[in] filename Source JSON file path.
 * @param[out] lightings Output lighting collection filled from the file contents.
 */
bool loadJSON(const std::string& filename, Lightings& lightings);
}

}  // namespace lightingEstimation
}  // namespace aliceVision
