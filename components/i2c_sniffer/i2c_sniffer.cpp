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
  ESP_LOGI(TAG, "SDA: %d, SCL: %d", sda, scl);

  // Detect start condition: SDA gaat laag terwijl SCL hoog is
  if (prev_sda == true && sda == false && scl == true) {
    ESP_LOGI(TAG, "Start condition detected");
    receiving = true;
    bit_count = 0;
    byte_buf = 0;
  }

  // Detect stop condition: SDA gaat hoog terwijl SCL hoog is
  if (prev_sda == false && sda == true && scl == true) {
    ESP_LOGI(TAG, "Stop condition detected");
    receiving = false;
  }

  // Als we data ontvangen: lees bits op stijgende flank van SCL
  if (receiving && prev_scl == false && scl == true) {
    // Shift in de SDA bit
    byte_buf = (byte_buf << 1) | (sda ? 1 : 0);
    bit_count++;

    if (bit_count == 8) {
      ESP_LOGI(TAG, "Byte received: 0x%02X", byte_buf);
      bit_count = 0;
      byte_buf = 0;
      // Hier kan je wachten op ACK bit (9e bit), maar dat wordt iets complexer
    }
  }

  prev_sda = sda;
  prev_scl = scl;
}

}  // namespace i2c_sniffer
}  // namespace esphome
