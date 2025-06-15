#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace i2c_sniffer {

class I2CSniffer : public Component {
 public:
  void set_buffer_sensor(text_sensor::TextSensor *sensor) { this->buffer_sensor_ = sensor; }
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
  void set_buffer_size(int size) {
    this->buffer_size_ = size;
  }

 protected:
  int sda_pin_ = 25;
  int scl_pin_ = 21;
  int log_level_ = 3;
  int buffer_size_ = 16;

  bool prev_sda_ = true;
  bool prev_scl_ = true;

  bool receiving_ = false;
  bool receiving_address_ = false;
  bool ack_bit_expected_ = false;

  uint8_t bit_count_ = 0;
  uint8_t byte_buf_ = 0;
  std::deque<std::string> buffer_;
  text_sensor::TextSensor *buffer_sensor_{nullptr};
};

void register_i2c_sniffer(int sda_pin, int scl_pin, int log_level = 3, int buffer_size = 16);

}  // namespace i2c_sniffer
}  // namespace esphome
