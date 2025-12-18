
#pragma once

#include <Eigen/Dense>
#include <string>

#include <aliceVision/sfmData/SharedPtrMap.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision {
namespace lightingEstimation {

class CalibrationSphere
{
  public:
    CalibrationSphere(const Eigen::Vector3f& center_, float radius_, bool is2D_=false, std::string type_ = std::string("matte"));
    
    const Eigen::Vector3f& getCenter() const;

    float getRadius() const;

    bool is2D() const;

  private:
    Eigen::Vector3f center;
    float radius;
    bool is2d; // True if only x and y are used
    std::string type;
};

/// Define a collection of calibration spheres (set of spheres per view)
using CalibrationSpheres = sfmData::SharedPtrMap<std::vector<CalibrationSphere>>;

namespace CalibrationSphereIO {
bool load(CalibrationSpheres& sphereData, const std::string& filename);
}
}  // namespace lightingEstimation
}  // namespace aliceVision
