#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include <Arduino.h>

namespace esphome {
namespace i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

#define SDA_PIN 21
#define SCL_PIN 25

// Houd vorige pinwaarden bij
bool prev_sda = true;
bool prev_scl = true;

// Buffers
uint8_t bit_count = 0;
uint8_t byte_buf = 0;
bool receiving = false;

// Voor check constant high line (pull-up check)
unsigned long last_change_time_sda = 0;
unsigned long last_change_time_scl = 0;
const unsigned long timeout_ms = 1000;  // 1 seconde

void I2CSniffer::setup() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  ESP_LOGI(TAG, "I2C Sniffer started");
  prev_sda = digitalRead(SDA_PIN);
  prev_scl = digitalRead(SCL_PIN);
  last_change_time_sda = millis();
  last_change_time_scl = millis();
}

void I2CSniffer::loop() {
  bool sda = digitalRead(SDA_PIN);
  bool scl = digitalRead(SCL_PIN);
  ESP_LOGI(TAG, "SDA: %d, SCL: %d", sda, scl);

  // Detect start condition
  if (prev_sda == true && sda == false && scl == true) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    receiving_address = true;  // volgende byte is adres
    bit_count = 0;
    byte_buf = 0;
  }

  // Detect stop condition
  if (prev_sda == false && sda == true && scl == true) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
  }

  if (receiving && prev_scl == false && scl == true) {
    // Bits inlezen
    byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
    bit_count++;

    if (bit_count == 8) {
      // 8 bits gelezen = 1 byte
      if (receiving_address) {
        // Adresbyte: hoogste 7 bits adres, laagste bit R/W
        uint8_t address = byte_buf >> 1;
        bool rw = byte_buf & 1;
        ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, rw ? "read (master ← slave)" : "write (master → slave)");
        receiving_address = false;
      } else {
        ESP_LOGI(TAG, "Data byte: 0x%02X", byte_buf);
      }
      bit_count = 0;
      byte_buf = 0;
      ack_bit_expected = true;  // 9e bit is ACK/NACK
    }
    else if (bit_count == 9 && ack_bit_expected) {
      // ACK bit lezen (1 = NACK, 0 = ACK)
      ESP_LOGI(TAG, "ACK bit: %d (%s)", sda ? 1 : 0, sda ? "NACK" : "ACK");
      ack_bit_expected = false;
      // Als ACK is, volgende byte is data, anders mogelijk stop condition
      if (!receiving_address) {
        // Na data byte weer data bytes verwacht
        // receiving_address blijft false, want adres was al binnen
      }
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}
}  // namespace i2c_sniffer
}  // namespace esphome
