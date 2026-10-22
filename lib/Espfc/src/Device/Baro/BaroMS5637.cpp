#include "BaroMS5637.hpp"
#include "Hal/Time.hpp"

namespace Espfc::Device::Baro {

#define MS5637_ADDRESS 0x76

#define MS5637_CMD_RESET 0x1E
#define MS5637_CMD_ADC_READ 0x00
#define MS5637_CMD_PROM_READ 0xA0     // + 2 * word index
#define MS5637_CMD_CONVERT_D1_8192 0x4A // pressure
#define MS5637_CMD_CONVERT_D2_2048 0x56 // temperature

// max conversion time + margin [us]
#define MS5637_DELAY_D1_8192 17000
#define MS5637_DELAY_D2_2048 4500

int BaroMS5637::begin(Hal::BusDevice* bus)
{
  return begin(bus, MS5637_ADDRESS);
}

int BaroMS5637::begin(Hal::BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  _bus->write(_addr, MS5637_CMD_RESET, 0, nullptr);
  delay(3);

  if (!readProm(_prom) || crc4(_prom) != (_prom[0] >> 12)) return 0;

  return 1;
}

BaroDeviceType BaroMS5637::getType() const
{
  return BARO_MS5637;
}

float BaroMS5637::readTemperature()
{
  const uint32_t d2 = readAdc();
  if (d2 == 0) return _temperature; // conversion not ready, keep last value
  _d2 = d2;

  int32_t temp, press; // temperature does not depend on d1
  compensate(_prom, _d1, _d2, temp, press);
  _temperature = temp * 0.01f;
  return _temperature;
}

float BaroMS5637::readPressure()
{
  const uint32_t d1 = readAdc();
  if (d1 == 0 || _d2 == 0) return _pressure; // conversion not ready, keep last value
  _d1 = d1;

  int32_t temp, press;
  compensate(_prom, _d1, _d2, temp, press);
  _temperature = temp * 0.01f;
  _pressure = press;
  return _pressure;
}

void BaroMS5637::setMode(BaroDeviceMode mode)
{
  const uint8_t cmd = mode == BARO_MODE_TEMP ? MS5637_CMD_CONVERT_D2_2048 : MS5637_CMD_CONVERT_D1_8192;
  _bus->write(_addr, cmd, 0, nullptr);
}

int BaroMS5637::getDelay(BaroDeviceMode mode) const
{
  return mode == BARO_MODE_TEMP ? MS5637_DELAY_D2_2048 : MS5637_DELAY_D1_8192;
}

bool BaroMS5637::testConnection()
{
  // no id register, a prom with a valid crc identifies the chip
  uint16_t prom[PROM_SIZE];
  if (!readProm(prom)) return false;
  const uint8_t crc = prom[0] >> 12;
  setChipId(crc);
  for (size_t i = 1; i < PROM_SIZE; i++)
  {
    if (prom[i] == 0x0000 || prom[i] == 0xFFFF) return false; // empty bus or wrong chip
  }
  return crc4(prom) == crc;
}

bool BaroMS5637::readProm(uint16_t* prom)
{
  for (size_t i = 0; i < PROM_SIZE; i++)
  {
    uint8_t buf[2] = {};
    if (_bus->read(_addr, MS5637_CMD_PROM_READ + i * 2, 2, buf) != 2) return false;
    prom[i] = (buf[0] << 8) | buf[1];
  }
  return true;
}

uint32_t BaroMS5637::readAdc()
{
  uint8_t buf[3] = {};
  if (_bus->read(_addr, MS5637_CMD_ADC_READ, 3, buf) != 3) return 0;
  return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

uint8_t BaroMS5637::crc4(const uint16_t* prom)
{
  uint16_t words[8];
  for (size_t i = 0; i < PROM_SIZE; i++) words[i] = prom[i];
  words[0] &= 0x0FFF; // crc bits
  words[7] = 0;

  uint16_t rem = 0;
  for (size_t cnt = 0; cnt < 16; cnt++)
  {
    rem ^= (cnt & 1) ? (words[cnt >> 1] & 0x00FF) : (words[cnt >> 1] >> 8);
    for (size_t bit = 8; bit > 0; bit--)
    {
      rem = (rem & 0x8000) ? (rem << 1) ^ 0x3000 : (rem << 1);
    }
  }
  return (rem >> 12) & 0x0F;
}

void BaroMS5637::compensate(const uint16_t* prom, uint32_t d1, uint32_t d2, int32_t& temp, int32_t& press)
{
  const int64_t c1 = prom[1], c2 = prom[2], c3 = prom[3], c4 = prom[4], c5 = prom[5], c6 = prom[6];

  const int64_t dT = (int64_t)d2 - (c5 << 8);
  int64_t t = 2000 + ((dT * c6) >> 23);
  int64_t off = (c2 << 17) + ((c4 * dT) >> 6);
  int64_t sens = (c1 << 16) + ((c3 * dT) >> 7);

  int64_t t2, off2 = 0, sens2 = 0;
  if (t < 2000)
  {
    const int64_t dt2 = (t - 2000) * (t - 2000);
    t2 = (3 * dT * dT) >> 33;
    off2 = (61 * dt2) >> 4;
    sens2 = (29 * dt2) >> 4;
    if (t < -1500)
    {
      const int64_t dt3 = (t + 1500) * (t + 1500);
      off2 += 17 * dt3;
      sens2 += 9 * dt3;
    }
  }
  else
  {
    t2 = (5 * dT * dT) >> 38;
  }

  t -= t2;
  off -= off2;
  sens -= sens2;

  temp = (int32_t)t;
  press = (int32_t)((((int64_t)d1 * sens >> 21) - off) >> 15);
}

} // namespace Espfc::Device::Baro
