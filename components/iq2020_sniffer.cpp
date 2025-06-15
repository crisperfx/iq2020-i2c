#include "iq2020_sniffer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iq2020_sniffer {

static const char *const TAG = "iq2020.sniffer";

#define SDA_PIN 25
#define SCL_PIN 21

void IQ2020Sniffer::setup() {
  ESP_LOGI(TAG, "Sniffer started (SDA=%d, SCL=%d)", SDA_PIN, SCL_PIN);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
}

void IQ2020Sniffer::loop() {
  bool sda = digitalRead(SDA_PIN);
  bool scl = digitalRead(SCL_PIN);
  ESP_LOGD(TAG, "SDA: %d, SCL: %d", sda, scl);
  delay(5);
}

}  // namespace iq2020_sniffer
}  // namespace esphome
