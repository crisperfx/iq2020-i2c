#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/application.h"
#include <Arduino.h>

namespace esphome {
namespace i2c_sniffer {

// Globale pointer declareren
I2CSniffer *global_sniffer = nullptr;

static const char *const TAG = "i2c_sniffer";

static bool stable_read(uint8_t pin) {
  bool v1 = digitalRead(pin);
  delayMicroseconds(5);
  bool v2 = digitalRead(pin);
  delayMicroseconds(5);
  bool v3 = digitalRead(pin);
  return (v1 == v2) && (v2 == v3);
}
void register_i2c_sniffer(int sda_pin, int scl_pin, int log_level, int buffer_size) {
  if (!global_sniffer) {
    global_sniffer = new I2CSniffer();
    global_sniffer->set_pins(sda_pin, scl_pin);
    global_sniffer->set_log_level(log_level);
    global_sniffer->set_buffer_size(buffer_size);
    App.register_component(global_sniffer);
  }
}

void I2CSniffer::setup() {
  pinMode(sda_pin_, INPUT_PULLUP);
  pinMode(scl_pin_, INPUT_PULLUP);
  ESP_LOGCONFIG("i2c_sniffer", "I2C Sniffer setup: SDA=%d, SCL=%d", sda_pin_, scl_pin_);

  prev_sda_ = digitalRead(sda_pin_);
  prev_scl_ = digitalRead(scl_pin_);
}

void I2CSniffer::loop() {
  bool sda = digitalRead(sda_pin_);
  bool scl = digitalRead(scl_pin_);

  // Detect start condition (SDA goes low while SCL high)
  if (prev_sda_ && !sda && scl) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving_ = true;
    receiving_address_ = true;
    bit_count_ = 0;
    byte_buf_ = 0;
    ack_bit_expected_ = false;
  }

  // Detect stop condition (SDA goes high while SCL high)
  if (!prev_sda_ && sda && scl) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving_ = false;
  }

  // Read bits on rising edge of SCL
  if (receiving_ && !prev_scl_ && scl) {
    if (ack_bit_expected_) {
      // Dit is het ACK/NACK bit, geen data
      if (sda == 0) {
        ESP_LOGD(TAG, "ACK received");
      } else {
        ESP_LOGD(TAG, "NACK received");
      }
      ack_bit_expected_ = false;
      bit_count_ = 0;
      byte_buf_ = 0;
      // Na ACK wordt volgende byte ontvangen, eventueel adres of data
    } else {
      // Shift in volgende data bit
      byte_buf_ = (byte_buf_ << 1) | (sda ? 1 : 0);
      bit_count_++;

      if (bit_count_ == 8) {
        if (receiving_address_) {
          uint8_t address = byte_buf_ >> 1;
          bool rw = byte_buf_ & 1;
          ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, rw ? "read (master ← slave)" : "write (master → slave)");
          receiving_address_ = false;  // volgende bytes zijn data
        } else {
          ESP_LOGI(TAG, "Data byte: 0x%02X", byte_buf_);
        }
        ack_bit_expected_ = true;  // verwacht nu ACK bit
      }
    }
  }

  prev_sda_ = sda;
  prev_scl_ = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
