#include "esphome/core/component.h"

namespace esphome {
namespace i2c_sniffer {

class I2CSniffer : public Component {
 public:
  void setup() override;
  void loop() override;
};

}  // namespace i2c_sniffer
}  // namespace esphome
