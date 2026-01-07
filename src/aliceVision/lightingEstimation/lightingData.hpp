#pragma once

#include <Eigen/Dense>

#include <aliceVision/sfmData/SharedPtrMap.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision {
namespace lightingEstimation {

enum class LightType
{
    Directionnal,
    Ponctual
};

class Lighting
{
  public:
    Lighting(LightType lightType_);

    virtual ~Lighting() = default;

    virtual Lighting* clone() const = 0;

    LightType getLightType() const;

  private:
    LightType lightType;
};

class DirectionnalLighting : public Lighting
{
  public:
    DirectionnalLighting(Eigen::Vector3f lightDirection_, float intensity_);

    ~DirectionnalLighting();

    DirectionnalLighting* clone() const { return new DirectionnalLighting(*this); }

  private:
    Eigen::Vector3f lightDirection;
    float intensity;
};

using Lightings = sfmData::SharedPtrMap<Lighting>;

}  // namespace lightingEstimation
}  // namespace aliceVision
