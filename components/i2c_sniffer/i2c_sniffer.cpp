#include "i2c_sniffer.h"
#include "esphome/core/log.h"
#include <Arduino.h>

namespace esphome {
namespace i2c_sniffer {

static const char *const TAG = "i2c_sniffer";

#define SDA_PIN 25
#define SCL_PIN 21

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

  // Detect verandering SDA
  if (sda != prev_sda) {
    ESP_LOGI(TAG, "SDA changed from %d to %d", prev_sda, sda);
    last_change_time_sda = millis();
  }

  // Detect verandering SCL
  if (scl != prev_scl) {
    ESP_LOGI(TAG, "SCL changed from %d to %d", prev_scl, scl);
    last_change_time_scl = millis();
  }

  // Check of SDA lang constant hoog blijft
  if (sda == true && millis() - last_change_time_sda > timeout_ms) {
    ESP_LOGW(TAG, "SDA line constant HIGH for more than %d ms. Check pull-ups and wiring!", timeout_ms);
  }

  // Check of SCL lang constant hoog blijft
  if (scl == true && millis() - last_change_time_scl > timeout_ms) {
    ESP_LOGW(TAG, "SCL line constant HIGH for more than %d ms. Check pull-ups and wiring!", timeout_ms);
  }

  // Detect start condition: SDA gaat van hoog naar laag terwijl SCL hoog is
  if (prev_sda == true && sda == false && scl == true) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    bit_count = 0;
    byte_buf = 0;
  }

  // Detect stop condition: SDA gaat van laag naar hoog terwijl SCL hoog is
  if (prev_sda == false && sda == true && scl == true) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
  }

  // Als we data ontvangen: lees bits op stijgende flank van SCL
  if (receiving && prev_scl == false && scl == true) {
    byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
    bit_count++;

    if (bit_count == 8) {
      ESP_LOGI(TAG, "Byte received: 0x%02X", byte_buf);
      bit_count = 0;
      byte_buf = 0;
      // ACK bit wordt niet gecontroleerd in deze simpele versie
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
