#include "i2c_sniffer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

#define SDA_PIN 25
#define SCL_PIN 21

void I2CSniffer::setup() {
  ESP_LOGI(TAG, "Sniffer setup START");
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  ESP_LOGI(TAG, "Sniffer setup COMPLETE");
}

void I2CSniffer::loop() {
  bool sda = digitalRead(SDA_PIN);
  bool scl = digitalRead(SCL_PIN);
  ESP_LOGD(TAG, "SDA: %d, SCL: %d", sda, scl);
}

// 👇 Automatisch registreren
static I2CSniffer *sniffer = new I2CSniffer();
ESPHOME_COMPONENT_REGISTER(sniffer);

}  // namespace i2c_sniffer
}  // namespace esphome
