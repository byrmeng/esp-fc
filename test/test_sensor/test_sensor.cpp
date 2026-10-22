#include "Sensor/VoltageSensor.hpp"
#include "Sensor/GyroBiasTracker.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Espfc::Sensor;

void setUpBattery(Model& model)
{
  model.state.battery.rawVoltage = 2000;
  model.state.battery.voltageUnfiltered = 12.4f;
  model.state.battery.voltage = 12.4f;
  model.state.battery.cellVoltage = 4.1f;
  model.state.battery.percentage = 90.0f;
  model.state.battery.cells = 3;
  model.state.battery.rawCurrent = 1000;
  model.state.battery.currentUnfiltered = 5.5f;
  model.state.battery.current = 5.5f;
}

void test_voltage_sensor_adc_change_reset_disabled()
{
  Model model;
  VoltageSensor sensor(model);
  setUpBattery(model);
  model.config.vbat.source = VBAT_SOURCE_NONE;
  model.config.ibat.source = IBAT_SOURCE_NONE;

  sensor.reload(MODEL_CHANGE_ADC);

  TEST_ASSERT_EQUAL_INT16(0, model.state.battery.rawVoltage);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.voltageUnfiltered);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.voltage);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.cellVoltage);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.percentage);
  TEST_ASSERT_EQUAL_INT8(0, model.state.battery.cells);
  TEST_ASSERT_EQUAL_INT16(0, model.state.battery.rawCurrent);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.currentUnfiltered);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.current);
}

void test_voltage_sensor_adc_change_keep_enabled()
{
  Model model;
  VoltageSensor sensor(model);
  setUpBattery(model);
  model.config.vbat.source = VBAT_SOURCE_ADC;
  model.config.ibat.source = IBAT_SOURCE_ADC;

  sensor.reload(MODEL_CHANGE_ADC);

  TEST_ASSERT_EQUAL_INT16(2000, model.state.battery.rawVoltage);
  TEST_ASSERT_EQUAL_FLOAT(12.4f, model.state.battery.voltage);
  TEST_ASSERT_EQUAL_INT8(3, model.state.battery.cells);
  TEST_ASSERT_EQUAL_INT16(1000, model.state.battery.rawCurrent);
  TEST_ASSERT_EQUAL_FLOAT(5.5f, model.state.battery.current);
  TEST_ASSERT_EQUAL_INT8(50, model.state.battery.samples); // cell count redetection
}

void test_voltage_sensor_adc_change_mixed()
{
  Model model;
  VoltageSensor sensor(model);
  setUpBattery(model);
  model.config.vbat.source = VBAT_SOURCE_ADC;
  model.config.ibat.source = IBAT_SOURCE_NONE;

  sensor.reload(MODEL_CHANGE_ADC);

  TEST_ASSERT_EQUAL_FLOAT(12.4f, model.state.battery.voltage);
  TEST_ASSERT_EQUAL_INT16(0, model.state.battery.rawCurrent);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.current);
}

void test_gyro_bias_tracker_waits_hold_time()
{
  GyroBiasTracker tracker;
  tracker.begin(1000.0f, 10.0f, 0.5f);
  VectorFloat bias(0.01f, 0.0f, 0.0f);
  const VectorFloat adc(0.02f, 0.0f, 0.0f);

  for (int i = 0; i < 500; i++) TEST_ASSERT_FALSE(tracker.update(bias, adc, 0.0f, false));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, bias.x);

  TEST_ASSERT_TRUE(tracker.update(bias, adc, 0.0f, false));
  TEST_ASSERT_GREATER_THAN_FLOAT(0.01f, bias.x);
}

void test_gyro_bias_tracker_follows_drift()
{
  GyroBiasTracker tracker;
  tracker.begin(1000.0f, 10.0f, 0.5f);
  VectorFloat bias(0.01f, -0.01f, 0.0f);
  const VectorFloat adc(0.02f, -0.02f, 0.01f); // drifted by 0.01 rad/s on each axis

  for (int i = 0; i < 500 + 10000; i++) tracker.update(bias, adc, 0.0f, false); // hold + one tau

  // one time constant closes ~63% of the 0.01 rad/s offset
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 0.01632f, bias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, -0.01632f, bias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 0.00632f, bias.z);
}

void test_gyro_bias_tracker_ignores_motion_and_armed()
{
  GyroBiasTracker tracker;
  tracker.begin(1000.0f, 10.0f, 0.0f);
  VectorFloat bias;

  TEST_ASSERT_FALSE(tracker.update(bias, VectorFloat(0.1f, 0.0f, 0.0f), 0.0f, false)); // rotating
  TEST_ASSERT_FALSE(tracker.update(bias, VectorFloat(0.01f, 0.0f, 0.0f), 0.1f, false)); // accel moving
  TEST_ASSERT_FALSE(tracker.update(bias, VectorFloat(0.01f, 0.0f, 0.0f), 0.0f, true));  // armed
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, bias.x);

  TEST_ASSERT_TRUE(tracker.update(bias, VectorFloat(0.01f, 0.0f, 0.0f), 0.0f, false));
}

void test_gyro_bias_tracker_motion_restarts_hold()
{
  GyroBiasTracker tracker;
  tracker.begin(1000.0f, 10.0f, 0.5f);
  VectorFloat bias;
  const VectorFloat still(0.01f, 0.0f, 0.0f);

  for (int i = 0; i < 500; i++) tracker.update(bias, still, 0.0f, false);
  tracker.update(bias, VectorFloat(0.5f, 0.0f, 0.0f), 0.0f, false); // bump
  for (int i = 0; i < 500; i++) TEST_ASSERT_FALSE(tracker.update(bias, still, 0.0f, false));
  TEST_ASSERT_TRUE(tracker.update(bias, still, 0.0f, false));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_voltage_sensor_adc_change_reset_disabled);
  RUN_TEST(test_voltage_sensor_adc_change_keep_enabled);
  RUN_TEST(test_voltage_sensor_adc_change_mixed);
  RUN_TEST(test_gyro_bias_tracker_waits_hold_time);
  RUN_TEST(test_gyro_bias_tracker_follows_drift);
  RUN_TEST(test_gyro_bias_tracker_ignores_motion_and_armed);
  RUN_TEST(test_gyro_bias_tracker_motion_restarts_hold);

  return UNITY_END();
}
