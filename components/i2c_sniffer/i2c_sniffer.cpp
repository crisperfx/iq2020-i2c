#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include <Arduino.h>

namespace esphome {
namespace i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

void I2CSniffer::setup() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  ESP_LOGI(TAG, "I2C Sniffer started");
  prev_sda = digitalRead(SDA_PIN);
  prev_scl = digitalRead(SCL_PIN);
  receiving = false;
  receiving_address = false;
  ack_bit_expected = false;
  bit_count = 0;
  byte_buf = 0;
}

void I2CSniffer::loop() {
  bool sda = digitalRead(SDA_PIN);
  bool scl = digitalRead(SCL_PIN);

  // Detect start condition (SDA hoog->laag terwijl SCL hoog)
  if (prev_sda && !sda && scl) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    receiving_address = true;
    bit_count = 0;
    byte_buf = 0;
    ack_bit_expected = false;
  }

  // Detect stop condition (SDA laag->hoog terwijl SCL hoog)
  if (!prev_sda && sda && scl) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
  }

  if (receiving && !prev_scl && scl) {  // stijgende flank SCL
    if (ack_bit_expected) {
      ESP_LOGI(TAG, "ACK bit: %d", sda ? 1 : 0);
      ack_bit_expected = false;
      if (receiving_address) {
        receiving_address = false;  // daarna volgen data bytes
      }
    } else {
      byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
      bit_count++;

      if (bit_count == 8) {
        if (receiving_address) {
          uint8_t address = byte_buf >> 1;
          bool rw = byte_buf & 1;
          ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, rw ? "read (master ← slave)" : "write (master → slave)");
        } else {
          ESP_LOGI(TAG, "Data byte: 0x%02X", byte_buf);
        }
        bit_count = 0;
        byte_buf = 0;
        ack_bit_expected = true;
      }
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
