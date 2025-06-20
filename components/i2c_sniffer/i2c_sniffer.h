#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <deque>
#include <set>
#include <string>

namespace esphome {
namespace i2c_sniffer {

class I2CSniffer : public Component {
 public:
  void set_buffer_sensor(text_sensor::TextSensor *sensor);
  void publish_buffer();
  void setup() override;
  void loop() override;

  void set_pins(int sda, int scl) {
    this->sda_pin_ = sda;
    this->scl_pin_ = scl;
  }

  void set_log_level(int level) {
    this->log_level_ = level;
  }

  void set_buffer_size(size_t size) {
    this->max_buffer_size_ = size;
  }

 protected:
  // Configuratie
  int sda_pin_ = 25;
  int scl_pin_ = 21;
  int log_level_ = 3;
  size_t max_buffer_size_ = 16;

  // Interne status
  bool prev_sda_ = true;
  bool prev_scl_ = true;

  bool receiving_ = false;
  bool receiving_address_ = false;
  bool ack_bit_expected_ = false;

  uint8_t bit_count_ = 0;
  uint8_t byte_buf_ = 0;

  std::deque<std::string> buffer_;
  text_sensor::TextSensor *buffer_sensor_{nullptr};

  // Bitlogging
  uint32_t last_scl_rise_us_ = 0;
  std::string bitstream_;
  bool start_detected_ = false;
  bool stop_detected_ = false;

  // I2C adresscanner
  std::set<uint8_t> detected_addresses_;

  void log_bit(bool bit) {
    uint32_t now = micros();
    uint32_t delta = now - last_scl_rise_us_;
    last_scl_rise_us_ = now;

    char buf[24];
    snprintf(buf, sizeof(buf), "%d:Δ%lu ", bit ? 1 : 0, (unsigned long) delta);
    bitstream_ += buf;

    if (bitstream_.length() > 256)
      bitstream_.erase(0, bitstream_.length() - 256);
  }

  void handle_byte(uint8_t byte_value, bool is_address);
  void handle_start_condition();
  void handle_stop_condition();
  void publish_detected_addresses();
};

void register_i2c_sniffer(int sda_pin, int scl_pin, int log_level = 3, int buffer_size = 16);

}  // namespace i2c_sniffer
}  // namespace esphome
