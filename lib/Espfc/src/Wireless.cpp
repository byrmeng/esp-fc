#include "Wireless.h"
#include "Hal/Platform.hpp"
#include <cstdio>

#ifdef ESPFC_SERIAL_SOFT_0_WIFI

namespace Espfc {

Wireless::Wireless(Model& model): _model(model), _status(STOPPED), _adapter(*Hal::getWifiClient()) {}

int Wireless::begin()
{
  Hal::Wifi::begin();
#ifdef ESPFC_HAL_ESPNOW
  if (_model.isFeatureActive(FEATURE_RX_SPI))
  {
    startAp();
  }
#endif
  return 1;
}

void Wireless::startAp()
{
#ifdef ESPFC_WIFI_AP_SSID_PREFIX
  uint8_t mac[6] = {0};
  Hal::Wifi::getApMac(mac);
  char ssid[33];
  snprintf(ssid, sizeof(ssid), "%s-%02X%02X", ESPFC_WIFI_AP_SSID_PREFIX, mac[4], mac[5]);
#else
  const char* ssid = "ESP-FC";
#endif
#ifdef ESPFC_WIFI_AP_PASS
  const char* pass = ESPFC_WIFI_AP_PASS;
#else
  const char* pass = nullptr;
#endif
  bool status = Hal::Wifi::startAp(ssid, pass);
  _model.logger.info().log("WIFI AP START").log(ssid).logln(status);
}

int Wireless::connect()
{
  Hal::Wifi::setListener(this);
  if (_model.config.wireless.ssid[0] != 0)
  {
    Hal::Wifi::startSta(_model.config.wireless.ssid, _model.config.wireless.pass);
    Hal::WifiStatus status;
    Hal::Wifi::getStatus(status);
    _model.logger.info()
        .log("WIFI STA")
        .log(_model.config.wireless.ssid)
        .log(_model.config.wireless.pass)
        .log((int)status.mode)
        .logln(status.state);
  }
  if (!Hal::Wifi::isApActive())
  {
    startAp();
  }
  Hal::Wifi::listen(_model.config.wireless.port);
  _model.state.serial[SERIAL_SOFT_0].stream = &_adapter;
  _model.logger.info().log("WIFI SERVER PORT").logln(_model.config.wireless.port);
  return 1;
}

void Wireless::onStaConnected(const char* ssid, int channel)
{
  _model.logger.info().log("WIFI STA CONN").log(ssid).logln(channel);
}

void Wireless::onApStaConnected(const uint8_t* mac)
{
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  _model.logger.info().log("WIFI AP CONNECT").logln(buf);
}

void Wireless::onStaGotIp(const char* ip)
{
  _model.logger.info().log("WIFI STA IP").logln(ip);
}

void Wireless::onStaDisconnected()
{
  _model.logger.info().logln("WIFI STA DISCONNECT");
}

int Wireless::update()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_WIFI);

  switch (_status)
  {
    case STOPPED:
      if (_model.state.mode.rescueConfigMode == RESCUE_CONFIG_ACTIVE && _model.isFeatureActive(FEATURE_SOFTSERIAL))
      {
        connect();
        _status = STARTED;
        return 1;
      }
      break;
    case STARTED:
      Hal::Wifi::accept();
      break;
  }

  return 1;
}

} // namespace Espfc

#endif
