#pragma once
#include "esphome/core/component.h"

namespace esphome {
namespace i2c_sniffer {

class I2CSniffer : public Component {
 public:
  void setup() override;
  void loop() override;

 private:
  // Houd vorige pinwaarden bij
  bool prev_sda = true;
  bool prev_scl = true;
  static const uint8_t SDA_PIN = 25;
  static const uint8_t SCL_PIN = 21;
  // Status flags en buffers
  bool receiving = false;
  bool receiving_address = false;
  bool ack_bit_expected = false;
  uint8_t bit_count = 0;
  uint8_t byte_buf = 0;
};

}  // namespace i2c_sniffer
}  // namespace esphome
