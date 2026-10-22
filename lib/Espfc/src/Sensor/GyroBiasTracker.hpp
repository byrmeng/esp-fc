#pragma once

#include <helper_3dmath.hpp>

namespace Espfc::Sensor {

// Slowly follows gyro zero-rate drift (e.g. thermal) while disarmed and held still.
// Startup calibration takes the bias in the first seconds, before the board warms up.
class GyroBiasTracker
{
public:
  static constexpr float ACCEL_STILL = 0.05f; // accel change between samples [g]
  static constexpr float GYRO_STILL = 0.035f; // bias corrected rate, ~2 deg/s [rad/s]

  void begin(float rate, float tau = 10.0f, float holdTime = 0.5f)
  {
    _alpha = rate > 0 ? 1.0f / (rate * tau) : 0.0f;
    _holdSamples = (int)(rate * holdTime);
    _stillSamples = 0;
  }

  // adc: gyro rate before bias correction [rad/s]
  // returns true when bias was updated
  bool update(VectorFloat& bias, VectorFloat adc, float accelDelta, bool armed)
  {
    VectorFloat rate = adc - bias;
    if (armed || accelDelta >= ACCEL_STILL || rate.getMagnitude() >= GYRO_STILL)
    {
      _stillSamples = 0;
      return false;
    }
    if (_stillSamples < _holdSamples)
    {
      _stillSamples++;
      return false;
    }
    bias += rate * _alpha;
    return true;
  }

private:
  float _alpha = 0.0f;
  int _holdSamples = 0;
  int _stillSamples = 0;
};

} // namespace Espfc::Sensor
