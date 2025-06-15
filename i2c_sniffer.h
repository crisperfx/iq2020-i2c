#include "esphome.h"

class I2CSniffer : public Component {
 public:
  int sda_pin = 25;
  int scl_pin = 21;

  void setup() override {
    ESP_LOGI("i2c_sniffer", "I2C Sniffer initialized on SDA=%d, SCL=%d", sda_pin, scl_pin);
    pinMode(sda_pin, INPUT_PULLUP);
    pinMode(scl_pin, INPUT_PULLUP);
  }

  void loop() override {
    // Simpele sniffer: toont status van SDA/SCL elke loop (kan worden uitgebreid)
    bool sda_state = digitalRead(sda_pin);
    bool scl_state = digitalRead(scl_pin);
    ESP_LOGD("i2c_sniffer", "SDA: %d, SCL: %d", sda_state, scl_state);
    delay(5);  // Verlaag delay voor sneller sniffen
  }
};
