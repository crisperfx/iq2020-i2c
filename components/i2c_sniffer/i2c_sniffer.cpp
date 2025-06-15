#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include <Arduino.h>

namespace esphome {
namespace i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

#define SDA_PIN 25
#define SCL_PIN 21

bool prev_sda = true;
bool prev_scl = true;

uint8_t bit_count = 0;
uint8_t byte_buf = 0;
bool receiving = false;
bool ack_phase = false;
bool expect_address = false;

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

  // Start condition (SDA valt terwijl SCL hoog is)
  if (prev_sda == true && sda == false && scl == true) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    ack_phase = false;
    bit_count = 0;
    byte_buf = 0;
    expect_address = true;
  }

  // Stop condition (SDA stijgt terwijl SCL hoog is)
  if (prev_sda == false && sda == true && scl == true) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
    ack_phase = false;
    bit_count = 0;
    byte_buf = 0;
  }

  // Alleen data lezen als we "ontvangen"
  if (receiving) {
    // Detecteer stijgende flank van SCL
    if (prev_scl == false && scl == true) {
      if (!ack_phase) {
        // Data bit inlezen
        byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
        bit_count++;

        if (bit_count == 8) {
          ESP_LOGI(TAG, "Byte received: 0x%02X", byte_buf);
          bit_count = 0;
          ack_phase = true;
        }
        if (expect_address) {
          uint8_t address = byte_buf >> 1;
          bool is_read = byte_buf & 0x01;
          ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, is_read ? "read" : "write");
          expect_address = false;
        } else {
          ESP_LOGI(TAG, "Data byte: 0x%02X", byte_buf);
        }

      } else {
        ESP_LOGI(TAG, "ACK bit: %s", sda ? "NACK" : "ACK");
        ack_phase = false;
        byte_buf = 0;
      }
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
