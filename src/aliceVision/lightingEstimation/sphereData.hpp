
#pragma once

#include <Eigen/Dense>
#include <string>

#include <aliceVision/sfmData/SharedPtrMap.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision {
namespace lightingEstimation {

/// Describe a calibration sphere used during lighting estimation.
class CalibrationSphere
{
  public:
    /**
     * @brief Construct a calibration sphere description.
     * @param[in] center_ Sphere center in image or scene space.
     * @param[in] radius_ Sphere radius.
     * @param[in] is2D_ True when only the x and y components of the center are meaningful.
     * @param[in] type_ Sphere material or category identifier.
     */
    CalibrationSphere(const Eigen::Vector3f& center_, float radius_, bool is2D_=false, std::string type_ = std::string("matte"));
    
    /**
     * @brief Get the sphere center.
     * @return Constant reference to the sphere center.
     */
    const Eigen::Vector3f& getCenter() const;

    /**
     * @brief Get the sphere radius.
     * @return Sphere radius.
     */
    float getRadius() const;

    /**
     * @brief Indicate whether the sphere is handled as a 2D object.
     * @return True if only the x and y coordinates are used.
     */
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
/**
 * @brief Load calibration spheres from a JSON file.
 * @param[out] sphereData Output collection filled with the spheres indexed by view identifier.
 * @param[in] filename Path to the JSON file describing the calibration spheres.
 * @return True if the file was parsed and the sphere data was loaded.
 */
bool load(CalibrationSpheres& sphereData, const std::string& filename);
}
}  // namespace lightingEstimation
}  // namespace aliceVision
