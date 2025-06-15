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
bool is_read_operation = false;

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

  // Detecteer startconditie
  if (prev_sda == true && sda == false && scl == true) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    bit_count = 0;
    byte_buf = 0;
    expect_address = true;
    ack_phase = false;
  }

  // Detecteer stopconditie
  if (prev_sda == false && sda == true && scl == true) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
  }

  // Verwerk bit op stijgende flank van SCL
  if (receiving && prev_scl == false && scl == true) {
    if (!ack_phase) {
      // Verzamel bits in byte_buf
      byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
      bit_count++;

      if (bit_count == 8) {
        if (expect_address) {
          uint8_t address = byte_buf >> 1;
          is_read_operation = byte_buf & 0x01;
          ESP_LOGI(TAG, "Address byte: 0x%02X (%s)", address, is_read_operation ? "read (master ← slave)" : "write (master → slave)");
          expect_address = false;
        } else {
          ESP_LOGI(TAG, "Data byte: 0x%02X (%s)", byte_buf, is_read_operation ? "slave → master" : "master → slave");
        }
        ack_phase = true;
        bit_count = 0;
      }
    } else {
      // 9e bit = ACK/NACK
      if (sda == 0) {
        ESP_LOGI(TAG, "ACK received");
      } else {
        ESP_LOGI(TAG, "NACK received");
      }
      ack_phase = false;
      byte_buf = 0;
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
