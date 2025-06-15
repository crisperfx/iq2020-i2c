#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace i2c_sniffer {

void register_i2c_sniffer(int sda_pin, int scl_pin, int log_level = 3, int buffer_size = 16);

class I2CSniffer : public Component {
 public:
  void setup() override;
  void loop() override;

  void set_pins(int sda_pin, int scl_pin) {
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
  }

 protected:
  int sda_pin_ = 25;
  int scl_pin_ = 21;

  bool prev_sda_ = true;
  bool prev_scl_ = true;

  bool receiving_ = false;
  bool receiving_address_ = false;
  bool ack_bit_expected_ = false;

  uint8_t bit_count_ = 0;
  uint8_t byte_buf_ = 0;
};

}  // namespace i2c_sniffer
}  // namespace esphome
