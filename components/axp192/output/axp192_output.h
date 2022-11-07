#pragma once

#include "../axp192.h"
#include "esphome/components/output/float_output.h"

namespace esphome {
namespace axp192 {

class AXP192Component;

class AXP192ComponentOutput : public output::FloatOutput, public Component {
 public:
  void set_parent(AXP192Component *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  void write_state(float state) override;

  AXP192Component *parent_;
  uint8_t pin_;
};

}  // namespace axp192
}  // namespace esphome