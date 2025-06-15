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
}

void I2CSniffer::loop() {
  bool sda = digitalRead(SDA_PIN);
  bool scl = digitalRead(SCL_PIN);

  // Log lines voor debug (kan je uitzetten om log spam te vermijden)
  // ESP_LOGI(TAG, "SDA: %d, SCL: %d", sda, scl);

  // Start condition: SDA gaat van hoog naar laag terwijl SCL hoog is
  if (prev_sda == true && sda == false && scl == true) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    receiving_address = true;
    ack_bit_expected = false;
    bit_count = 0;
    byte_buf = 0;
  }

  // Stop condition: SDA gaat van laag naar hoog terwijl SCL hoog is
  if (prev_sda == false && sda == true && scl == true) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
  }

  // Data bits lezen op stijgende flank van SCL
  if (receiving && prev_scl == false && scl == true) {
    if (ack_bit_expected) {
      // ACK bit ontvangen
      if (sda == 0) {
        ESP_LOGI(TAG, "ACK received");
      } else {
        ESP_LOGI(TAG, "NACK received");
      }
      ack_bit_expected = false;
      receiving_address = false;  // na adres is het data
      bit_count = 0;
      byte_buf = 0;
    } else {
      // Bits in byte schuiven
      byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
      bit_count++;

      if (bit_count == 8) {
        if (receiving_address) {
          // Adresbyte: hoogste 7 bits adres, LSB R/W
          uint8_t address = byte_buf >> 1;
          bool read_write = byte_buf & 1;
          ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, read_write ? "read (master ← slave)" : "write (master → slave)");
        } else {
          ESP_LOGI(TAG, "Data byte: 0x%02X", byte_buf);
        }
        ack_bit_expected = true;  // Nu volgt het ack bit
      }
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
