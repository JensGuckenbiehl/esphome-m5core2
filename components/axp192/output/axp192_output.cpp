#include "axp192_output.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace axp192 {

static const char *const TAG = "axp192_output";

void AXP192ComponentOutput::write_state(float state) {
    ESP_LOGD(TAG, "write state %f", state);
    this->parent_->SetBrightness(state);
}

void AXP192ComponentOutput::setup() {
    this->turn_on();
    //this->set_level(0.5f);
}

void AXP192ComponentOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "  axp192 pin: %d", this->pin_);
  LOG_FLOAT_OUTPUT(this);
}

}  // namespace axp192
}  // namespace esphome
