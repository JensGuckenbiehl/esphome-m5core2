#include "axp192.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esp_sleep.h"

namespace esphome {
namespace axp192 {

static const char *TAG = "axp192.sensor";
void AXP192Component::setup() 
{
  switch (this->model_) {
    case AXP192_M5STICKC:
    {
        // NOT SUPPORTED
    }
    case AXP192_M5CORE2:
    {
        begin();
    }
  }
}

void AXP192Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP192:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
}

float AXP192Component::get_setup_priority() const { return setup_priority::HARDWARE; }

void AXP192Component::update() {

    if (this->batterylevel_sensor_ != nullptr) {
      // To be fixed
      // This is not giving the right value - mostly there to have some sample sensor...
      float vbat = GetBatVoltage();
      float batterylevel = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));

      ESP_LOGD(TAG, "Got Battery Level=%f (%f)", batterylevel, vbat);
      if (batterylevel > 100.) {
        batterylevel = 100;
      }
      this->batterylevel_sensor_->publish_state(batterylevel);
    }

    UpdateBrightness();

    if(curr_sound_ != sound_) {
      curr_sound_ = sound_;
      SetSpkEnable(curr_sound_);
    }
}

void AXP192Component::UpdateBrightness()
{
    ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
    if (brightness_ == curr_brightness_)
    {
        return;
    }
    curr_brightness_ = brightness_;

    ScreenBreath(brightness_ * 100);
   if(brightness_ <= 0.01) {
    SetDCDC3(false);
  }
}

void AXP192Component::begin() {
  //AXP192 30H
  Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0X02);

  //AXP192 GPIO1:OD OUTPUT
  Write1Byte(0x92, Read8bit(0x92) & 0xf8);

  //AXP192 GPIO2:OD OUTPUT
  Write1Byte(0x93, Read8bit(0x93) & 0xf8);

  //AXP192 RTC CHG
  Write1Byte(0x35, (Read8bit(0x35) & 0x1c) | 0xa2);

  SetESPVoltage(3350);

  SetLcdVoltage(2800);

  SetLDOVoltage(2, 3300);  //Periph power voltage preset (LCD_logic, SD card)

  SetLDOVoltage(3, 2000);  //Vibrator power voltage preset

  SetLDOEnable(2, true);
  SetDCDC3(true);  // LCD backlight
  SetLed(true);

  SetCHGCurrent(kCHG_100mA);
  //SetAxpPriphPower(1);
  //Serial.printf("axp: lcd_logic and sdcard power enabled\n\n");

  //pinMode(39, INPUT_PULLUP);

  //AXP192 GPIO4
  Write1Byte(0X95, (Read8bit(0x95) & 0x72) | 0X84);

  Write1Byte(0X36, 0X4C);

  Write1Byte(0x82, 0xff);

  SetLCDRSet(0);
  delay(100);
  SetLCDRSet(1);
  delay(100);
  // I2C_WriteByteDataAt(0X15,0XFE,0XFF);

  SetPeripherialsPower(true);

  // axp: check v-bus status
  if (true || (Read8bit(0x00) & 0x08)) {
    Write1Byte(0x30, Read8bit(0x30) | 0x80);
    // if v-bus can use, disable M-Bus 5V output to input
    SetBusPowerMode(kMBusModeInput);
  } else {
    // if not, enable M-Bus 5V output
    SetBusPowerMode(kMBusModeOutput);
  }
  
  SetSpkEnable(sound_);
  curr_sound_ = sound_;
  
  ScreenBreath(brightness_ * 100);
  if(brightness_ <= 0.01) {
    SetDCDC3(false);
  }
}
void AXP192Component::Write1Byte( uint8_t Addr ,  uint8_t Data )
{
    this->write_byte(Addr, Data);
}

uint8_t AXP192Component::Read8bit( uint8_t Addr )
{
    uint8_t data;
    this->read_byte(Addr, &data);
    return data;
}

uint16_t AXP192Component::Read12Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 4) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read13Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 5) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read16bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read24bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read32bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP192Component::ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff )
{
    this->read_bytes(Addr, Buff, Size);
}

void AXP192Component::ScreenBreath(int brightness) {
  //int vol = map(brightness, 0, 100, 2400, 3300);
  int vol = (brightness * 900 / 100) + 2400;
  // Serial.printf("brightness:%d\n", brightness);

  // Serial.printf("vol:%d\n", vol);
  // Serial.printf("vol:%u\n", vol);

  SetLcdVoltage((uint16_t)vol);
  // delay(10);
  // uint8_t buf = Read8bit(0x27);
  // Serial.printf("brightness:%hhu\n", brightness);
  // Serial.printf("brightness:%d\n", brightness);
  // Serial.printf("brightness:%x\n", brightness);

  // Serial.printf("buf:%hhu\n", buf);
  // Serial.printf("buf:%d\n", buf);
  // Serial.printf("buf:%x\n", buf);

  // Serial.printf("result:%hhu\n", ((buf & 0x0f) | (brightness << 4)));
  // Serial.printf("result:%d\n", ((buf & 0x0f) | (brightness << 4)));
  // Serial.printf("result:%x\n", ((buf & 0x0f) | (brightness << 4)));

  // Write1Byte(0x27, ((buf & 0x0f) | (brightness << 4)));
}

bool AXP192Component::GetBatState() {
  if (Read8bit(0x01) | 0x20)
    return true;
  else
    return false;
}
//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void);
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void AXP192Component::EnableCoulombcounter(void) { Write1Byte(0xB8, 0x80); }

void AXP192Component::DisableCoulombcounter(void) { Write1Byte(0xB8, 0x00); }

void AXP192Component::StopCoulombcounter(void) { Write1Byte(0xB8, 0xC0); }

void AXP192Component::ClearCoulombcounter(void) { Write1Byte(0xB8, 0xA0); }

uint32_t AXP192Component::GetCoulombchargeData(void) { return Read32bit(0xB0); }

uint32_t AXP192Component::GetCoulombdischargeData(void) { return Read32bit(0xB4); }

float AXP192Component::GetCoulombData(void) {
  uint32_t coin = 0;
  uint32_t coout = 0;

  coin = GetCoulombchargeData();
  coout = GetCoulombdischargeData();

  //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
  //Adc rate can be read from 84H ,change this variable if you change the ADC reate
  float ccc = 65536 * 0.5 * (int32_t)(coin - coout) / 3600.0 / 25.0;
  return ccc;
}

// Cut all power, except for LDO1 (RTC)
void AXP192Component::PowerOff(void) { Write1Byte(0x32, Read8bit(0x32) | 0b10000000); }

void AXP192Component::SetAdcState(bool state) {
  // Enable / Disable all ADCs
  Write1Byte(0x82, state ? 0xff : 0x00);
}

void AXP192Component::PrepareToSleep(void) {
  // Disable ADCs
  SetAdcState(false);

  // Turn LED off
  SetLed(false);

  // Turn LCD backlight off
  SetDCDC3(false);
}

// Get current battery level
float AXP192Component::GetBatteryLevel(void) {
  const float batVoltage = GetBatVoltage();
  const float batPercentage =
      (batVoltage < 3.248088) ? (0) : (batVoltage - 3.120712) * 100;
  return (batPercentage <= 100) ? batPercentage : 100;
}

void AXP192Component::RestoreFromLightSleep(void) {
  // Turn LCD backlight on
  SetDCDC3(true);

  // Turn LED on
  SetLed(true);

  // Enable ADCs
  SetAdcState(true);
}

// -- sleep
void AXP192Component::DeepSleep(uint64_t time_in_us) {
  PrepareToSleep();

  if (time_in_us > 0) {
    esp_sleep_enable_timer_wakeup(time_in_us);
  } else {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  }
  (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);

  // Never reached - after deep sleep ESP32 restarts
}

void AXP192Component::LightSleep(uint64_t time_in_us) {
  PrepareToSleep();

  if (time_in_us > 0) {
    esp_sleep_enable_timer_wakeup(time_in_us);
  } else {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  }
  esp_light_sleep_start();

  RestoreFromLightSleep();
}

uint8_t AXP192Component::GetWarningLevel(void) { return Read8bit(0x47) & 0x01; }

float AXP192Component::GetBatVoltage() {
  float ADCLSB = 1.1 / 1000.0;
  uint16_t ReData = Read12Bit(0x78);
  return ReData * ADCLSB;
}

float AXP192Component::GetBatCurrent() {
  float ADCLSB = 0.5;
  uint16_t CurrentIn = Read13Bit(0x7A);
  uint16_t CurrentOut = Read13Bit(0x7C);
  return (CurrentIn - CurrentOut) * ADCLSB;
}

float AXP192Component::GetVinVoltage() {
  float ADCLSB = 1.7 / 1000.0;
  uint16_t ReData = Read12Bit(0x56);
  return ReData * ADCLSB;
}

float AXP192Component::GetVinCurrent() {
  float ADCLSB = 0.625;
  uint16_t ReData = Read12Bit(0x58);
  return ReData * ADCLSB;
}

float AXP192Component::GetVBusVoltage() {
  float ADCLSB = 1.7 / 1000.0;
  uint16_t ReData = Read12Bit(0x5A);
  return ReData * ADCLSB;
}

float AXP192Component::GetVBusCurrent() {
  float ADCLSB = 0.375;
  uint16_t ReData = Read12Bit(0x5C);
  return ReData * ADCLSB;
}

float AXP192Component::GetTempInAXP192() {
  float ADCLSB = 0.1;
  const float OFFSET_DEG_C = -144.7;
  uint16_t ReData = Read12Bit(0x5E);
  return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192Component::GetBatPower() {
  float VoltageLSB = 1.1;
  float CurrentLCS = 0.5;
  uint32_t ReData = Read24bit(0x70);
  return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

float AXP192Component::GetBatChargeCurrent() {
  float ADCLSB = 0.5;
  uint16_t ReData = Read12Bit(0x7A);
  return ReData * ADCLSB;
}
float AXP192Component::GetAPSVoltage() {
  float ADCLSB = 1.4 / 1000.0;
  uint16_t ReData = Read12Bit(0x7E);
  return ReData * ADCLSB;
}

float AXP192Component::GetBatCoulombInput() {
  uint32_t ReData = Read32bit(0xB0);
  return ReData * 65536 * 0.5 / 3600 / 25.0;
}

float AXP192Component::GetBatCoulombOut() {
  uint32_t ReData = Read32bit(0xB4);
  return ReData * 65536 * 0.5 / 3600 / 25.0;
}

void AXP192Component::SetCoulombClear() { Write1Byte(0xB8, 0x20); }

void AXP192Component::SetLDO2(bool State) {
  uint8_t buf = Read8bit(0x12);
  if (State == true)
    buf = (1 << 2) | buf;
  else
    buf = ~(1 << 2) & buf;
  Write1Byte(0x12, buf);
}

void AXP192Component::SetDCDC3(bool State) {
  uint8_t buf = Read8bit(0x12);
  if (State == true)
    buf = (1 << 1) | buf;
  else
    buf = ~(1 << 1) & buf;
  Write1Byte(0x12, buf);
}

uint8_t AXP192Component::AXPInState() { return Read8bit(0x00); }
bool AXP192Component::isACIN() { return (Read8bit(0x00) & 0x80) ? true : false; }
bool AXP192Component::isCharging() { return (Read8bit(0x00) & 0x04) ? true : false; }
bool AXP192Component::isVBUS() { return (Read8bit(0x00) & 0x20) ? true : false; }

uint8_t calcVoltageData(uint16_t value, uint16_t maxv, uint16_t minv,
                        uint16_t step) {
  uint8_t data = 0;
  if (value > maxv) value = maxv;
  if (value > minv) data = (value - minv) / step;
  return data;
}

void AXP192Component::SetLDOVoltage(uint8_t number, uint16_t voltage) {
  uint8_t vdata = calcVoltageData(voltage, 3300, 1800, 100) & 0x0F;
  switch (number) {
    //uint8_t reg, data;
    case 2:
      Write1Byte(0x28, (Read8bit(0x28) & 0x0F) | (vdata << 4));
      break;
    case 3:
      Write1Byte(0x28, (Read8bit(0x28) & 0xF0) | vdata);
      break;
  }
}

/// @param number 0=DCDC1 / 1=DCDC2 / 2=DCDC3
void AXP192Component::SetDCVoltage(uint8_t number, uint16_t voltage) {
  uint8_t addr;
  uint8_t vdata;
  if (number > 2) return;
  switch (number) {
    case 0:
      addr = 0x26;
      vdata = calcVoltageData(voltage, 3500, 700, 25) & 0x7F;
      break;
    case 1:
      addr = 0x25;
      vdata = calcVoltageData(voltage, 2275, 700, 25) & 0x3F;
      break;
    case 2:
      addr = 0x27;
      vdata = calcVoltageData(voltage, 3500, 700, 25) & 0x7F;
      break;
  }
  // Serial.printf("result:%hhu\n", (Read8bit(addr) & 0X80) | (voltage & 0X7F));
  // Serial.printf("result:%d\n", (Read8bit(addr) & 0X80) | (voltage & 0X7F));
  // Serial.printf("result:%x\n", (Read8bit(addr) & 0X80) | (voltage & 0X7F));
  Write1Byte(addr, (Read8bit(addr) & 0x80) | vdata);
}

void AXP192Component::SetESPVoltage(uint16_t voltage) {
  if (voltage >= 3000 && voltage <= 3400) {
    SetDCVoltage(0, voltage);
  }
}

void AXP192Component::SetLcdVoltage(uint16_t voltage) {
  if (voltage >= 2500 && voltage <= 3300) {
    SetDCVoltage(2, voltage);
  }
}

void AXP192Component::SetLDOEnable(uint8_t number, bool state) {
  uint8_t mark = 0x01;
  if ((number < 2) || (number > 3)) return;

  mark <<= number;
  if (state) {
    Write1Byte(0x12, (Read8bit(0x12) | mark));
  } else {
    Write1Byte(0x12, (Read8bit(0x12) & (~mark)));
  }
}

void AXP192Component::SetLCDRSet(bool state) {
  uint8_t reg_addr = 0x96;
  uint8_t gpio_bit = 0x02;
  uint8_t data;
  data = Read8bit(reg_addr);

  if (state) {
    data |= gpio_bit;
  } else {
    data &= ~gpio_bit;
  }

  Write1Byte(reg_addr, data);
}

// Select source for BUS_5V
// 0 : use internal boost
// 1 : powered externally
void AXP192Component::SetBusPowerMode(uint8_t state) {
  uint8_t data;
  if (state == 0) {
    // Set GPIO to 3.3V (LDO OUTPUT mode)
    data = Read8bit(0x91);
    Write1Byte(0x91, (data & 0x0F) | 0xF0);
    // Set GPIO0 to LDO OUTPUT, pullup N_VBUSEN to disable VBUS supply from BUS_5V
    data = Read8bit(0x90);
    Write1Byte(0x90, (data & 0xF8) | 0x02);
    // Set EXTEN to enable 5v boost
    data = Read8bit(0x10);
    Write1Byte(0x10, data | 0x04);
  } else {
    // Set EXTEN to disable 5v boost
    data = Read8bit(0x10);
    Write1Byte(0x10, data & ~0x04);
    // Set GPIO0 to float, using enternal pulldown resistor to enable VBUS supply from BUS_5V
    data = Read8bit(0x90);
    Write1Byte(0x90, (data & 0xF8) | 0x07);
  }
}

void AXP192Component::SetLed(uint8_t state) {
  uint8_t reg_addr = 0x94;
  uint8_t data;
  data = Read8bit(reg_addr);

  if (state) {
    data = data & 0XFD;
  } else {
    data |= 0X02;
  }

  Write1Byte(reg_addr, data);
}

//set led state(GPIO high active,set 1 to enable amplifier)
void AXP192Component::SetSpkEnable(uint8_t state) {
  uint8_t reg_addr = 0x94;
  uint8_t gpio_bit = 0x04;
  uint8_t data;
  data = Read8bit(reg_addr);

  if (state) {
    data |= gpio_bit;
  } else {
    data &= ~gpio_bit;
  }

  Write1Byte(reg_addr, data);
}

void AXP192Component::SetCHGCurrent(uint8_t state) {
  uint8_t data = Read8bit(0x33);
  data &= 0xf0;
  data = data | (state & 0x0f);
  Write1Byte(0x33, data);
}

void AXP192Component::SetPeripherialsPower(uint8_t state) {
  if (!state)
    Write1Byte(0x10, Read8bit(0x10) & 0XFB);
  else if (state)
    Write1Byte(0x10, Read8bit(0x10) | 0X04);
  uint8_t data;
  // Set EXTEN to enable 5v boost
}

}
}

