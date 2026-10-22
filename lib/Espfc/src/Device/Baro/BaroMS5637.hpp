#pragma once

#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

// TE MS5637-02BA03, I2C only, fixed address 0x76
class BaroMS5637 : public BaroDevice
{
public:
  static constexpr size_t PROM_SIZE = 7; // CRC/factory word + C1..C6

  int begin(Hal::BusDevice* bus) final;
  int begin(Hal::BusDevice* bus, uint8_t addr) final;

  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode) final;
  int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

  // CRC4 from TE AN520, stored in the top 4 bits of prom[0]
  static uint8_t crc4(const uint16_t* prom);

  // datasheet first and second order compensation
  // d1, d2: raw pressure and temperature, temp: 0.01 degC, press: Pa
  static void compensate(const uint16_t* prom, uint32_t d1, uint32_t d2, int32_t& temp, int32_t& press);

private:
  bool readProm(uint16_t* prom);
  uint32_t readAdc();

  uint16_t _prom[PROM_SIZE] = {};
  uint32_t _d1 = 0;
  uint32_t _d2 = 0;
  float _temperature = 0.0f;
  float _pressure = 0.0f;
};

} // namespace Espfc::Device::Baro
