#pragma once

#include <cmath>
#include <helper_3dmath.hpp>

namespace Espfc::Sensor {

// Accel calibration takes samples only while the board rests: accel barely changes between samples,
// its magnitude is close to 1g and the gyro rate is small.
class StillDetector
{
public:
  static constexpr float ACCEL_DELTA = 0.05f;          // accel change between samples [m/s^2]
  static constexpr float ACCEL_NORM_TOLERANCE = 0.1f; // allowed deviation of |accel| from 1g [g]
  static constexpr float GYRO_RATE = 0.1f;             // bias corrected rate, ~6 deg/s [rad/s]

  // accel: uncorrected accel [m/s^2], gyro: bias corrected rate [rad/s]
  static bool isStill(VectorFloat accel, VectorFloat prev, VectorFloat gyro, float gravity)
  {
    return (accel - prev).getMagnitude() < ACCEL_DELTA &&
           std::abs(accel.getMagnitude() - gravity) < ACCEL_NORM_TOLERANCE * gravity &&
           gyro.getMagnitude() < GYRO_RATE;
  }
};

} // namespace Espfc::Sensor
