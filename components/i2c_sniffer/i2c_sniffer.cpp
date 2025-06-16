#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/application.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <Arduino.h>
#include <string>
#include <vector>

namespace esphome {
namespace i2c_sniffer {

// Globale pointer declareren
I2CSniffer *global_sniffer = nullptr;

static const char *const TAG = "i2c_sniffer";

// Helper: converteer byte naar hex string
static std::string to_hex(uint8_t byte) {
  const char* hex_chars = "0123456789ABCDEF";
  std::string hex;
  hex += hex_chars[(byte >> 4) & 0x0F];
  hex += hex_chars[byte & 0x0F];
  return hex;
}

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
  ESP_LOGCONFIG(TAG, "I2C Sniffer setup: SDA=%d, SCL=%d", sda_pin_, scl_pin_);

  prev_sda_ = digitalRead(sda_pin_);
  prev_scl_ = digitalRead(scl_pin_);
  last_scl_rise_us_ = micros();

  receiving_ = false;
  receiving_address_ = false;
  ack_bit_expected_ = false;
  bit_count_ = 0;
  byte_buf_ = 0;
  bitstream_.clear();
  buffer_.clear();
}

void I2CSniffer::publish_buffer() {
  if (this->buffer_sensor_ == nullptr)
    return;

  std::string buffer_str;
  for (size_t i = 0; i < this->buffer_.size(); ++i) {
    std::string hex_val = this->buffer_[i];
    if (hex_val.compare(0, 2, "0x") == 0 || hex_val.compare(0, 2, "0X") == 0) {
      hex_val = hex_val.substr(2);
    }
    buffer_str += hex_val;
    if (i != this->buffer_.size() - 1)
      buffer_str += " ";
  }

  if (!bitstream_.empty()) {
    buffer_str += " | Bits: " + bitstream_;
    bitstream_.clear();
  }

  // Limiteer lengte buffer string
  if (buffer_str.length() > 512)
    buffer_str = buffer_str.substr(buffer_str.length() - 512);

  this->buffer_sensor_->publish_state(buffer_str);
  ESP_LOGI(TAG, "Publishing buffer: %s", buffer_str.c_str());

  // Optioneel clear buffer na publiceren (kan ook later)
  buffer_.clear();
}

void I2CSniffer::set_buffer_sensor(text_sensor::TextSensor *sensor) {
  this->buffer_sensor_ = sensor;
}

void I2CSniffer::loop() {
  bool sda = digitalRead(sda_pin_);
  bool scl = digitalRead(scl_pin_);

  // Detecteer startconditie (SDA valt terwijl SCL hoog)
  if (prev_sda_ && !sda && scl) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving_ = true;
    receiving_address_ = true;
    bit_count_ = 0;
    byte_buf_ = 0;
    ack_bit_expected_ = false;
    bitstream_ += "[START] ";
    last_scl_rise_us_ = micros();
  }

  // Detecteer stopconditie (SDA stijgt terwijl SCL hoog)
  if (!prev_sda_ && sda && scl) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving_ = false;
    bitstream_ += " [STOP]\n";
  }

  // Bits loggen op stijgende SCL-flank
  if (receiving_ && !prev_scl_ && scl) {
    uint32_t now = micros();
    uint32_t delta = now - last_scl_rise_us_;
    last_scl_rise_us_ = now;

    bitstream_ += (sda ? "1" : "0");
    bitstream_ += ":Δ" + std::to_string(delta) + " ";

    if (ack_bit_expected_) {
      if (sda == 0) {
        ESP_LOGD(TAG, "ACK received");
      } else {
        ESP_LOGD(TAG, "NACK received");
      }
      ack_bit_expected_ = false;
      bit_count_ = 0;
      byte_buf_ = 0;
    } else {
      byte_buf_ = (byte_buf_ << 1) | (sda ? 1 : 0);
      bit_count_++;

      if (bit_count_ == 8) {
        std::string byte_str = "0x" + to_hex(byte_buf_);
        buffer_.push_back(byte_str);

        if (buffer_.size() > max_buffer_size_)
          buffer_.pop_front();

        if (receiving_address_) {
          uint8_t address = byte_buf_ >> 1;
          bool rw = byte_buf_ & 1;
          ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, rw ? "read (master ← slave)" : "write (master → slave)");
          receiving_address_ = false;
        } else {
          ESP_LOGI(TAG, "Data byte: 0x%02X", byte_buf_);
        }
        ack_bit_expected_ = true;
      }
    }
  }

  // Periodiek publiceren, bv. elke 2 seconden
  static uint32_t last_publish = 0;
  if (millis() - last_publish > 2000) {
    last_publish = millis();
    this->publish_buffer();
  }

  prev_sda_ = sda;
  prev_scl_ = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
