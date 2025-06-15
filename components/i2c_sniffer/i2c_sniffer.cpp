#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "Arduino.h"

namespace esphome {
namespace i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

#define SDA_PIN 25
#define SCL_PIN 21

// Houd vorige pinwaarden bij
bool prev_sda = true;
bool prev_scl = true;

// Buffers
uint8_t bit_count = 0;
uint8_t byte_buf = 0;
bool receiving = false;

void I2CSniffer::setup() {
  pinMode(this->sda_pin_, INPUT_PULLUP);
  pinMode(this->scl_pin_, INPUT_PULLUP);
  ESP_LOGI(TAG, "I2C sniffer setup complete on SDA pin %d and SCL pin %d", sda_pin_, scl_pin_);
}

void I2CSniffer::loop() {
  static bool prev_scl = true;
  static uint8_t bit_count = 0;
  static uint8_t current_byte = 0;

  bool sda = digitalRead(this->sda_pin_);
  bool scl = digitalRead(this->scl_pin_);

  if (prev_scl == false && scl == true) {
    // Rising edge detected → tijd om SDA te lezen
    current_byte = (current_byte << 1) | (sda ? 1 : 0);
    bit_count++;

    if (bit_count == 8) {
      ESP_LOGI(TAG, "Byte received: 0x%02X", current_byte);
      bit_count = 0;
      current_byte = 0;
    }
  }

  // Start condition: SDA daalt terwijl SCL hoog is
  if (scl && prev_sda_ && !sda) {
    ESP_LOGI(TAG, "Start condition detected");
  }

  // Stop condition: SDA stijgt terwijl SCL hoog is
  if (scl && !prev_sda_ && sda) {
    ESP_LOGI(TAG, "Stop condition detected");
  }

  prev_scl = scl;
  prev_sda_ = sda;
}


}  // namespace i2c_sniffer
}  // namespace esphome
