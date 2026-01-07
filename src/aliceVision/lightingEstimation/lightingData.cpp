#include "lightingData.hpp"

namespace aliceVision {
namespace lightingEstimation {

Lighting::Lighting(LightType lightType_)
  : lightType(lightType_)
{}

LightType Lighting::getLightType() const
{ return lightType; }


DirectionnalLighting::DirectionnalLighting(Eigen::Vector3f lightDirection_, float intensity_)
  : Lighting(LightType::Directionnal),
    lightDirection(lightDirection_),
    intensity(intensity_)
{}

DirectionnalLighting::~DirectionnalLighting()
{}

}  // namespace lightingEstimation
}  // namespace aliceVision
